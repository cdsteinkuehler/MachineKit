
//
//    Copyright (C) 2007-2008 Sebastian Kuzminsky
//
//    This program is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program; if not, write to the Free Software
//    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
//


//
//  This file contains the driver for the HostMot2 encoder v2 Module.
//
//  It supports Index and Index Mask, and high-fidelity velocity 
//  estimation.
//


#include "config_module.h"
#include RTAPI_INC_SLAB_H

#include "rtapi.h"
#include "rtapi_string.h"
#include "rtapi_math.h"

#include "hal.h"

#include "hal/drivers/mesa-hostmot2/hostmot2.h"




static void do_flag(u32 *reg, int condition, u32 bits) {
    if (condition) {
        *reg |= bits;
    } else {
        *reg &= ~bits;
    }
}


static void hm2_encoder_update_control_register(hostmot2_t *hm2,
						hm2_encoder_t *encoder) {
    int i;

    for (i = 0; i < encoder->num_instances; i ++) {
        hm2_encoder_instance_t *e = &encoder->instance[i];
        int index_enable = *e->hal.pin.index_enable;
        int latch_enable = *e->hal.pin.latch_enable;
        encoder->control_reg[i] = 0;

        do_flag(
            &encoder->control_reg[i],
            index_enable,
            HM2_ENCODER_LATCH_ON_INDEX
        );

        do_flag(
            &encoder->control_reg[i],
            !index_enable && latch_enable,
            HM2_ENCODER_LATCH_ON_PROBE
        );

        do_flag(
            &encoder->control_reg[i],
            index_enable || latch_enable,
	    HM2_ENCODER_INDEX_JUSTONCE
	);

        do_flag(
            &encoder->control_reg[i],
            *e->hal.pin.latch_polarity,
            HM2_ENCODER_PROBE_POLARITY
        );

        do_flag(
            &encoder->control_reg[i],
            e->hal.param.index_invert,
            HM2_ENCODER_INDEX_POLARITY
        );

        do_flag(
            &encoder->control_reg[i],
            e->hal.param.index_mask,
            HM2_ENCODER_INDEX_MASK
        );

        do_flag(
            &encoder->control_reg[i],
            e->hal.param.index_mask_invert,
            HM2_ENCODER_INDEX_MASK_POLARITY
        );

        do_flag(
            &encoder->control_reg[i],
            e->hal.param.counter_mode,
            HM2_ENCODER_COUNTER_MODE
        );

        do_flag(
            &encoder->control_reg[i],
            e->hal.param.filter,
            HM2_ENCODER_FILTER
        );
    }
}


static void hm2_encoder_read_control_register(hostmot2_t *hm2,
					      hm2_encoder_t *encoder) {
    int i;
    static int last_error_enable = 0;

    for (i = 0; i < encoder->num_instances; i ++) {
        hm2_encoder_instance_t *e = &encoder->instance[i];

        if (*e->hal.pin.quadrature_error_enable) {
            if (!last_error_enable) {
                hm2_encoder_force_write(hm2, encoder);
                last_error_enable = 1;
            } else {
                int state = (encoder->read_control_reg[i] & HM2_ENCODER_CONTROL_MASK) & HM2_ENCODER_QUADRATURE_ERROR;
                if ((*e->hal.pin.quadrature_error == 0) && state) {
                    HM2_ERR("Encoder %d: quadrature count error", i);
                }
                *e->hal.pin.quadrature_error = (hal_bit_t) state;
            }
        } else {
            *e->hal.pin.quadrature_error = 0;
            last_error_enable = 0;
        }

        *e->hal.pin.input_a = encoder->read_control_reg[i] & HM2_ENCODER_INPUT_A;
        *e->hal.pin.input_b = encoder->read_control_reg[i] & HM2_ENCODER_INPUT_B;
        *e->hal.pin.input_idx = encoder->read_control_reg[i] & HM2_ENCODER_INPUT_INDEX;
    }
}


static void hm2_encoder_set_filter_rate_and_skew(hostmot2_t *hm2,
						 hm2_encoder_t *encoder) {
    u32 filter_rate = encoder->clock_frequency/(*encoder->hal->pin.sample_frequency);
    
    if (filter_rate == 1) {
        filter_rate = 0xFFF;
    } else {
        filter_rate -= 2;
    }
    *encoder->hal->pin.sample_frequency = encoder->clock_frequency/(filter_rate + 2);
    HM2_DBG("Setting encoder QFilterRate to %d\n", filter_rate);
    if (encoder->has_skew) {
        u32 skew = (*encoder->hal->pin.skew)/(1e9/encoder->clock_frequency);
        
        if (skew > 15) {
            skew = 15;
        }
        HM2_DBG("Setting mux encoder skew to %d\n", skew);
        filter_rate |= (skew & 0xF) << 28;
        *encoder->hal->pin.skew = skew*(1e9/encoder->clock_frequency);
        encoder->written_skew = *encoder->hal->pin.skew;
    }
    hm2->llio->write(hm2->llio, encoder->filter_rate_addr, &filter_rate, sizeof(u32));
    encoder->written_sample_frequency = *encoder->hal->pin.sample_frequency;
}

void hm2_encoder_write(hostmot2_t *hm2,
		       hm2_encoder_t *encoder) {
    int i;

    if (encoder->num_instances == 0) return;

    hm2_encoder_update_control_register(hm2, encoder);

    for (i = 0; i < encoder->num_instances; i ++) {
        if ((encoder->instance[i].prev_control & HM2_ENCODER_CONTROL_MASK) != (encoder->control_reg[i] & HM2_ENCODER_CONTROL_MASK)) {
            goto force_write;
        }
    }

    if (*encoder->hal->pin.sample_frequency != encoder->written_sample_frequency) {
        goto force_write;
    }
    if (encoder->has_skew) {
        if (*encoder->hal->pin.skew != encoder->written_skew) {
            goto force_write;
        }
    }

    return;

force_write:
    hm2_encoder_force_write(hm2, encoder);
}


void hm2_encoder_force_write(hostmot2_t *hm2,
			     hm2_encoder_t *encoder) {
    int i;

    if (encoder->num_instances == 0) return;

    hm2_encoder_update_control_register(hm2, encoder);

    hm2->llio->write(
        hm2->llio,
        encoder->latch_control_addr,
        encoder->control_reg,
        (encoder->num_instances * sizeof(u32))
    );

    for (i = 0; i < encoder->num_instances; i ++) {
        encoder->instance[i].prev_control = encoder->control_reg[i];
    }

    hm2->llio->write(
        hm2->llio,
        encoder->timestamp_div_addr,
        &encoder->timestamp_div_reg,
        sizeof(u32)
    );

    hm2_encoder_set_filter_rate_and_skew(hm2, encoder);
}




int hm2_encoder_parse_md(hostmot2_t *hm2, hm2_encoder_t *encoder, int md_index, int num_encs) {
    hm2_module_descriptor_t *md = &hm2->md[md_index];
    int r;


    // 
    // some standard sanity checks
    //

    if (hm2->md[md_index].gtag == HM2_GTAG_ENCODER) {
        if (hm2_md_is_consistent(hm2, md_index, 2, 5, 4, 0x0003)) {
            // ok
        } else if (hm2_md_is_consistent_or_complain(hm2, md_index, 3, 5, 4, 0x0003)) {
            // ok
        } else {
            HM2_ERR("inconsistent Encoder Module Descriptor!\n");
            return -EINVAL;
        }
    } else if (hm2->md[md_index].gtag == HM2_GTAG_MUXED_ENCODER) {
        if (hm2_md_is_consistent(hm2, md_index, 2, 5, 4, 0x0003)) {
            HM2_PRINT("WARNING: this firmware has Muxed Encoder v2!\n");
            HM2_PRINT("WARNING: velocity computation will be incorrect!\n");
            HM2_PRINT("WARNING: upgrade your firmware!\n");
        } else if (hm2_md_is_consistent(hm2, md_index, 3, 5, 4, 0x0003)) {
            // ok
        } else if (hm2_md_is_consistent_or_complain(hm2, md_index, 4, 5, 4, 0x0003)) {
            // ok
        } else {
            HM2_ERR("inconsistent Muxed Encoder Module Descriptor!\n");
            return -EINVAL;
        }
    }

    if (encoder->num_instances != 0) {
        HM2_ERR(
            "found duplicate Module Descriptor for %s (inconsistent firmware), not loading driver\n",
            hm2_get_general_function_name(md->gtag)
        );
        return -EINVAL;
    }

    if (num_encs > md->instances) {
	HM2_ERR("config.num_%sencoders=%d, but only %d are available, not loading driver\n",
		(hm2->md[md_index].gtag == HM2_GTAG_ENCODER) ? ""  : "m",
		num_encs,
		md->instances);
        return -EINVAL;
    }

    if (num_encs == 0) {
        return 0;
    }


    //
    // looks good, start initializing
    // 


    if (num_encs == -1) {
        encoder->num_instances = md->instances;
    } else {
        encoder->num_instances = num_encs;
    }


    // allocate the module-global HAL shared memory
    encoder->hal = (hm2_encoder_module_global_t *)hal_malloc(sizeof(hm2_encoder_module_global_t));
    if (encoder->hal == NULL) {
        HM2_ERR("out of memory!\n");
        r = -ENOMEM;
        goto fail0;
    }

    encoder->instance = (hm2_encoder_instance_t *)hal_malloc(encoder->num_instances * sizeof(hm2_encoder_instance_t));
    if (encoder->instance == NULL) {
        HM2_ERR("out of memory!\n");
        r = -ENOMEM;
        goto fail0;
    }

    encoder->stride = md->register_stride;
    encoder->clock_frequency = md->clock_freq;
    encoder->version = md->version;

    encoder->counter_addr = md->base_address + (0 * md->register_stride);
    encoder->latch_control_addr = md->base_address + (1 * md->register_stride);
    encoder->timestamp_div_addr = md->base_address + (2 * md->register_stride);
    encoder->timestamp_count_addr = md->base_address + (3 * md->register_stride);
    encoder->filter_rate_addr = md->base_address + (4 * md->register_stride);

    // it's important that the TSC gets read *before* the C&T registers below
    r = hm2_register_tram_read_region(hm2, encoder->timestamp_count_addr, sizeof(u32), &encoder->timestamp_count_reg);
    if (r < 0) {
        HM2_ERR("error registering tram read region for Encoder Timestamp Count Register (%d)\n", r);
        goto fail0;
    }

    // it's important that the C&T registers get read *after* the TSC register above
    r = hm2_register_tram_read_region(hm2, encoder->counter_addr, (encoder->num_instances * sizeof(u32)), &encoder->counter_reg);
    if (r < 0) {
        HM2_ERR("error registering tram read region for Encoder Counter register (%d)\n", r);
        goto fail0;
    }

    r = hm2_register_tram_read_region(hm2, encoder->latch_control_addr, (encoder->num_instances * sizeof(u32)), &encoder->read_control_reg);
    if (r < 0) {
        HM2_ERR("error registering tram read region for Encoder Latch/Control register (%d)\n", r);
        goto fail0;
    }

    encoder->control_reg = (u32 *)kmalloc(encoder->num_instances * sizeof(u32), GFP_KERNEL);
    if (encoder->control_reg == NULL) {
        HM2_ERR("out of memory!\n");
        r = -ENOMEM;
        goto fail0;
    }


    // export the encoders to HAL
    // FIXME: r hides the r in enclosing function, and it returns the wrong thing
    {
        int i;
        int r;
        char name[HAL_NAME_LEN + 1];

        // these hal parameters affect all encoder instances
        if (md->gtag == HM2_GTAG_MUXED_ENCODER) {
            rtapi_snprintf(name, sizeof(name), "%s.encoder.muxed-sample-frequency", hm2->llio->name);
        } else {
            rtapi_snprintf(name, sizeof(name), "%s.encoder.sample-frequency", hm2->llio->name);
        }
        r = hal_pin_u32_new(name, HAL_IN, &(encoder->hal->pin.sample_frequency), hm2->llio->comp_id);
        if (r < 0) {
            HM2_ERR("error adding pin %s, aborting\n", name);
            goto fail1;
        }
        if ((md->gtag == HM2_GTAG_MUXED_ENCODER) && (md->version == 4)) {
            rtapi_snprintf(name, sizeof(name), "%s.encoder.muxed-skew", hm2->llio->name);
            r = hal_pin_u32_new(name, HAL_IN, &(encoder->hal->pin.skew), hm2->llio->comp_id);
            if (r < 0) {
                HM2_ERR("error adding pin %s, aborting\n", name);
                goto fail1;
            }
            encoder->has_skew = 1;
        }

        for (i = 0; i < encoder->num_instances; i ++) {
            // pins
            rtapi_snprintf(name, sizeof(name), "%s.encoder.%02d.rawcounts", hm2->llio->name, i + hm2->encoder_base);
            r = hal_pin_s32_new(name, HAL_OUT, &(encoder->instance[i].hal.pin.rawcounts), hm2->llio->comp_id);
            if (r < 0) {
                HM2_ERR("error adding pin '%s', aborting\n", name);
                goto fail1;
            }

            rtapi_snprintf(name, sizeof(name), "%s.encoder.%02d.rawlatch", hm2->llio->name, i + hm2->encoder_base);
            r = hal_pin_s32_new(name, HAL_OUT, &(encoder->instance[i].hal.pin.rawlatch), hm2->llio->comp_id);
            if (r < 0) {
                HM2_ERR("error adding pin '%s', aborting\n", name);
                goto fail1;
            }

            rtapi_snprintf(name, sizeof(name), "%s.encoder.%02d.count", hm2->llio->name, i + hm2->encoder_base);
            r = hal_pin_s32_new(name, HAL_OUT, &(encoder->instance[i].hal.pin.count), hm2->llio->comp_id);
            if (r < 0) {
                HM2_ERR("error adding pin '%s', aborting\n", name);
                goto fail1;
            }

            rtapi_snprintf(name, sizeof(name), "%s.encoder.%02d.count-latched", hm2->llio->name, i + hm2->encoder_base);
            r = hal_pin_s32_new(name, HAL_OUT, &(encoder->instance[i].hal.pin.count_latch), hm2->llio->comp_id);
            if (r < 0) {
                HM2_ERR("error adding pin '%s', aborting\n", name);
                goto fail1;
            }

            rtapi_snprintf(name, sizeof(name), "%s.encoder.%02d.position", hm2->llio->name, i + hm2->encoder_base);
            r = hal_pin_float_new(name, HAL_OUT, &(encoder->instance[i].hal.pin.position), hm2->llio->comp_id);
            if (r < 0) {
                HM2_ERR("error adding pin '%s', aborting\n", name);
                goto fail1;
            }

            rtapi_snprintf(name, sizeof(name), "%s.encoder.%02d.position-latched", hm2->llio->name, i + hm2->encoder_base);
            r = hal_pin_float_new(name, HAL_OUT, &(encoder->instance[i].hal.pin.position_latch), hm2->llio->comp_id);
            if (r < 0) {
                HM2_ERR("error adding pin '%s', aborting\n", name);
                goto fail1;
            }

            rtapi_snprintf(name, sizeof(name), "%s.encoder.%02d.velocity", hm2->llio->name, i + hm2->encoder_base);
            r = hal_pin_float_new(name, HAL_OUT, &(encoder->instance[i].hal.pin.velocity), hm2->llio->comp_id);
            if (r < 0) {
                HM2_ERR("error adding pin '%s', aborting\n", name);
                goto fail1;
            }

            rtapi_snprintf(name, sizeof(name), "%s.encoder.%02d.reset", hm2->llio->name, i + hm2->encoder_base);
            r = hal_pin_bit_new(name, HAL_IN, &(encoder->instance[i].hal.pin.reset), hm2->llio->comp_id);
            if (r < 0) {
                HM2_ERR("error adding pin '%s', aborting\n", name);
                goto fail1;
            }

            rtapi_snprintf(name, sizeof(name), "%s.encoder.%02d.index-enable", hm2->llio->name, i + hm2->encoder_base);
            r = hal_pin_bit_new(name, HAL_IO, &(encoder->instance[i].hal.pin.index_enable), hm2->llio->comp_id);
            if (r < 0) {
                HM2_ERR("error adding pin '%s', aborting\n", name);
                goto fail1;
            }

            rtapi_snprintf(name, sizeof(name), "%s.encoder.%02d.latch-enable", hm2->llio->name, i + hm2->encoder_base);
            r = hal_pin_bit_new(name, HAL_IN, &(encoder->instance[i].hal.pin.latch_enable), hm2->llio->comp_id);
            if (r < 0) {
                HM2_ERR("error adding pin '%s', aborting\n", name);
                goto fail1;
            }

            rtapi_snprintf(name, sizeof(name), "%s.encoder.%02d.latch-polarity", hm2->llio->name, i + hm2->encoder_base);
            r = hal_pin_bit_new(name, HAL_IN, &(encoder->instance[i].hal.pin.latch_polarity), hm2->llio->comp_id);
            if (r < 0) {
                HM2_ERR("error adding pin '%s', aborting\n", name);
                goto fail1;
            }

            rtapi_snprintf(name, sizeof(name), "%s.encoder.%02d.quad-error", hm2->llio->name, i + hm2->encoder_base);
            r = hal_pin_bit_new(name, HAL_OUT, &(encoder->instance[i].hal.pin.quadrature_error), hm2->llio->comp_id);
            if (r < 0) {
                HM2_ERR("error adding pin '%s', aborting\n", name);
                goto fail1;
            }

            rtapi_snprintf(name, sizeof(name), "%s.encoder.%02d.quad-error-enable", hm2->llio->name, i + hm2->encoder_base);
            r = hal_pin_bit_new(name, HAL_IN, &(encoder->instance[i].hal.pin.quadrature_error_enable), hm2->llio->comp_id);
            if (r < 0) {
                HM2_ERR("error adding pin '%s', aborting\n", name);
                goto fail1;
            }

            rtapi_snprintf(name, sizeof(name), "%s.encoder.%02d.input-a", hm2->llio->name, i + hm2->encoder_base);
            r = hal_pin_bit_new(name, HAL_OUT, &(encoder->instance[i].hal.pin.input_a), hm2->llio->comp_id);
            if (r < 0) {
                HM2_ERR("error adding pin '%s', aborting\n", name);
                goto fail1;
            }

            rtapi_snprintf(name, sizeof(name), "%s.encoder.%02d.input-b", hm2->llio->name, i + hm2->encoder_base);
            r = hal_pin_bit_new(name, HAL_OUT, &(encoder->instance[i].hal.pin.input_b), hm2->llio->comp_id);
            if (r < 0) {
                HM2_ERR("error adding pin '%s', aborting\n", name);
                goto fail1;
            }

            rtapi_snprintf(name, sizeof(name), "%s.encoder.%02d.input-index", hm2->llio->name, i + hm2->encoder_base);
            r = hal_pin_bit_new(name, HAL_OUT, &(encoder->instance[i].hal.pin.input_idx), hm2->llio->comp_id);
            if (r < 0) {
                HM2_ERR("error adding pin '%s', aborting\n", name);
                goto fail1;
            }

            // parameters
            rtapi_snprintf(name, sizeof(name), "%s.encoder.%02d.scale", hm2->llio->name, i + hm2->encoder_base);
            r = hal_param_float_new(name, HAL_RW, &(encoder->instance[i].hal.param.scale), hm2->llio->comp_id);
            if (r < 0) {
                HM2_ERR("error adding param '%s', aborting\n", name);
                goto fail1;
            }

            rtapi_snprintf(name, sizeof(name), "%s.encoder.%02d.index-invert", hm2->llio->name, i + hm2->encoder_base);
            r = hal_param_bit_new(name, HAL_RW, &(encoder->instance[i].hal.param.index_invert), hm2->llio->comp_id);
            if (r < 0) {
                HM2_ERR("error adding param '%s', aborting\n", name);
                goto fail1;
            }

            rtapi_snprintf(name, sizeof(name), "%s.encoder.%02d.index-mask", hm2->llio->name, i + hm2->encoder_base);
            r = hal_param_bit_new(name, HAL_RW, &(encoder->instance[i].hal.param.index_mask), hm2->llio->comp_id);
            if (r < 0) {
                HM2_ERR("error adding param '%s', aborting\n", name);
                goto fail1;
            }

            rtapi_snprintf(name, sizeof(name), "%s.encoder.%02d.index-mask-invert", hm2->llio->name, i + hm2->encoder_base);
            r = hal_param_bit_new(name, HAL_RW, &(encoder->instance[i].hal.param.index_mask_invert), hm2->llio->comp_id);
            if (r < 0) {
                HM2_ERR("error adding param '%s', aborting\n", name);
                goto fail1;
            }

            rtapi_snprintf(name, sizeof(name), "%s.encoder.%02d.counter-mode", hm2->llio->name, i + hm2->encoder_base);
            r = hal_param_bit_new(name, HAL_RW, &(encoder->instance[i].hal.param.counter_mode), hm2->llio->comp_id);
            if (r < 0) {
                HM2_ERR("error adding param '%s', aborting\n", name);
                goto fail1;
            }

            rtapi_snprintf(name, sizeof(name), "%s.encoder.%02d.filter", hm2->llio->name, i + hm2->encoder_base);
            r = hal_param_bit_new(name, HAL_RW, &(encoder->instance[i].hal.param.filter), hm2->llio->comp_id);
            if (r < 0) {
                HM2_ERR("error adding param '%s', aborting\n", name);
                goto fail1;
            }

            rtapi_snprintf(name, sizeof(name), "%s.encoder.%02d.vel-timeout", hm2->llio->name, i + hm2->encoder_base);
            r = hal_param_float_new(name, HAL_RW, &(encoder->instance[i].hal.param.vel_timeout), hm2->llio->comp_id);
            if (r < 0) {
                HM2_ERR("error adding param '%s', aborting\n", name);
                goto fail1;
            }


            //
            // init the hal objects that need it
            // the things not initialized here will be set by hm2_encoder_tram_init()
            //

            *encoder->instance[i].hal.pin.reset = 0;
            *encoder->instance[i].hal.pin.index_enable = 0;

            encoder->instance[i].hal.param.scale = 1.0;
            encoder->instance[i].hal.param.index_invert = 0;
            encoder->instance[i].hal.param.index_mask = 0;
            encoder->instance[i].hal.param.index_mask_invert = 0;
            encoder->instance[i].hal.param.counter_mode = 0;
            encoder->instance[i].hal.param.filter = 1;
            encoder->instance[i].hal.param.vel_timeout = 0.5;

            encoder->instance[i].state = HM2_ENCODER_STOPPED;

        }
    }


    // 
    // Set the Timestamp Divisor Register
    // 
    // We want the timestamp to count as quickly as possible, so we get the
    // best temporal resolution.
    // 
    // But we want it to count slow enough that the 16-bit counter doesnt
    // overrun between successive calls to the servo thread (easy), and
    // even slower so that we can do good low-speed velocity estimation
    // (long between roll-overs).
    //
    // A resonably slow servo thread runs at 1 KHz.  A fast one runs at 10
    // KHz.  The actual servo period is unknown at loadtime, and is likely 
    // to fluctuate slightly when the system is under load.
    // 
    // Peter suggests a Quadrature Timestamp clock rate of 1 MHz.  This
    // means that a 1 KHz servo period sees about 1000 clocks per period.
    //     
    //
    // From the HM2 RegMap:
    // 
    //     Timestamp count rate is ClockLow/(TSDiv+2).
    //     Any divisor with MSb set = divide by 1
    // 
    // This gives us:
    // 
    //     rate = 1 MHz = 1e6 = ClockLow / (TSDiv+2)
    // 
    //     TSDiv+2 = ClockLow / 1e6
    // 
    //     TSDiv = (ClockLow / 1e6) - 2
    //
    //     seconds_per_clock = 1 / rate = (TSDiv+2) / ClockLow
    //
    //
    // The 7i43 has a 50 MHz ClockLow, giving TSDiv = 48 and 1 us/clock
    // The PCI cards have a 33 MHz ClockLow, giving TSDiv = 31 and again 1 us/clock
    //

    encoder->timestamp_div_reg = (encoder->clock_frequency / 1e6) - 2;
    encoder->seconds_per_tsdiv_clock = (double)(encoder->timestamp_div_reg + 2) / (double)encoder->clock_frequency;

    if (md->gtag == HM2_GTAG_ENCODER) {
        *encoder->hal->pin.sample_frequency = 25000000;
    } else {
        *encoder->hal->pin.sample_frequency = 8000000;
    }

    return encoder->num_instances;

fail1:
    kfree(encoder->control_reg);

fail0:
    encoder->num_instances = 0;
    return r;
}


void hm2_encoder_tram_init(hostmot2_t *hm2, hm2_encoder_t *encoder) {
    int i;

    if (encoder->num_instances <= 0) return;

    // "all time is now"
    encoder->prev_timestamp_count_reg = *encoder->timestamp_count_reg;

    // all the encoders start "stopped where they are"
    for (i = 0; i < encoder->num_instances; i ++) {
        u16 count;

        count = hm2_encoder_get_reg_count(encoder, i);

        *encoder->instance[i].hal.pin.rawcounts = count;
        *encoder->instance[i].hal.pin.rawlatch = count;

        *encoder->instance[i].hal.pin.count = 0;
        *encoder->instance[i].hal.pin.count_latch = 0;
        *encoder->instance[i].hal.pin.position = 0.0;
        *encoder->instance[i].hal.pin.position_latch = 0.0;
        *encoder->instance[i].hal.pin.velocity = 0.0;
        *encoder->instance[i].hal.pin.quadrature_error = 0;

        encoder->instance[i].zero_offset = count;

        encoder->instance[i].prev_reg_count = count;

        encoder->instance[i].state = HM2_ENCODER_STOPPED;

        // Note: we dont initialize the other internal state variables,
        // because in the "Stopped" state they're not used

    }
}




/**
 * @brief Updates the encoder's rawcounts accumulator, and checks for index pulse if appropriate.
 *
 * Sets these variables:
 *
 *     hal.pin.rawcounts
 *     .prev_reg_count
 *     (maybe) .index_enabled and .zero_offset
 *
 * This function expects the TRAM read to have just finished, so that
 * counter_reg[i] is up-to-date.
 *
 * May read the Latch register (if searching for the Index pulse).
 *
 * @param hm2 The hostmot2 structure being worked on.
 *
 * @param instance The index of the encoder instance.
 */

static void hm2_encoder_instance_update_rawcounts_and_handle_index(hostmot2_t *hm2,
								   hm2_encoder_t *encoder,
								   int instance) {
    u16 reg_count;
    s32 reg_count_diff;
    s32 prev_rawcounts;

    hm2_encoder_instance_t *e;

    e = &encoder->instance[instance];
    prev_rawcounts = *e->hal.pin.rawcounts;


    // 
    // figure out current rawcounts accumulated by the driver
    // 

    reg_count = hm2_encoder_get_reg_count(encoder, instance);

    reg_count_diff = (s32)reg_count - (s32)e->prev_reg_count;
    if (reg_count_diff > 32768) reg_count_diff -= 65536;
    if (reg_count_diff < -32768) reg_count_diff += 65536;

    *e->hal.pin.rawcounts += reg_count_diff;


    //
    // if we've told the FPGA we're looking for an index pulse:
    //     read the latch/ctrl register
    //     if it's triggered set zero_offset to the rawcounts version of the latched count
    //

    if (e->prev_control & HM2_ENCODER_LATCH_ON_INDEX) {
        u32 latch_ctrl;

        hm2->llio->read(
            hm2->llio,
            encoder->latch_control_addr + (instance * sizeof(u32)),
            &latch_ctrl,
            sizeof(u32)
        );

        if (0 == (latch_ctrl & HM2_ENCODER_LATCH_ON_INDEX)) {
            // hm2 reports index event occurred

            u16 latched_count;

            latched_count = (latch_ctrl >> 16) & 0xffff;

            reg_count_diff = (s32)latched_count - (s32)e->prev_reg_count;
            if (reg_count_diff > 32768) reg_count_diff -= 65536;
            if (reg_count_diff < -32768) reg_count_diff += 65536;

            e->zero_offset = prev_rawcounts + reg_count_diff;
            *e->hal.pin.index_enable = 0;
        }
    } else if(e->prev_control & HM2_ENCODER_LATCH_ON_PROBE) {
        u32 latch_ctrl;

        hm2->llio->read(
            hm2->llio,
            encoder->latch_control_addr + (instance * sizeof(u32)),
            &latch_ctrl,
            sizeof(u32)
        );

        if (0 == (latch_ctrl & HM2_ENCODER_LATCH_ON_PROBE)) {
            // hm2 reports probe event occurred

            u16 latched_count;

            latched_count = (latch_ctrl >> 16) & 0xffff;

            reg_count_diff = (s32)latched_count - (s32)e->prev_reg_count;
            if (reg_count_diff > 32768) reg_count_diff -= 65536;
            if (reg_count_diff < -32768) reg_count_diff += 65536;

            *(e->hal.pin.rawlatch) = prev_rawcounts + reg_count_diff;
            // *e->hal.pin.latch_enable = 0;
        }
    }

    e->prev_reg_count = reg_count;
}




/**
 * @brief Updates the driver's idea of the encoder's position.
 *
 * Sets these variables:
 *
 *     hal.pin.count
 *     hal.pin.position
 *     .prev_reg_count
 *     .zero_offset (if hal.pin.reset is True)
 *
 * This function expects the TRAM read and (if appropriate) the
 * hm2_encoder_instance_update_rawcounts_and_check_for_index() function to
 * have just finished, so that counter_reg[i], rawcounts, and zero_offset
 * are all up-to-date.
 *
 * This function optionally sets .zero_offset (if .reset is True), then
 * computes .counts and .position from .rawcounts, .scale, and
 * .zero_offset.
 *
 * @param hm2 The hostmot2 structure being worked on.
 *
 * @param instance The index of the encoder instance.
 */

static void hm2_encoder_instance_update_position(hostmot2_t *hm2,
						 hm2_encoder_t *encoder,
						 int instance) {

    hm2_encoder_instance_t *e = &encoder->instance[instance];


    // 
    // reset count if the user wants us to (by just setting the zero offset to the current rawcounts)
    //

    if (*e->hal.pin.reset) {
        e->zero_offset = *e->hal.pin.rawcounts;
    }


    // 
    // Now we know the current rawcounts and zero_offset, which together
    // tell us the current count.
    //
    // From that we easily compute count and scaled position.
    //

    *e->hal.pin.count = *e->hal.pin.rawcounts - e->zero_offset;
    *e->hal.pin.count_latch = *e->hal.pin.rawlatch - e->zero_offset;


    //
    // Now we know the current current .count, from this we easily compute
    // the scaled position.
    //

    *e->hal.pin.position = *e->hal.pin.count / e->hal.param.scale;
    *e->hal.pin.position_latch = *e->hal.pin.count_latch / e->hal.param.scale;
}




/**
 * @brief Processes the information received from the QuadratureCounter
 *     (encoder) instance to produce an estimate of position & velocity.
 */

static void hm2_encoder_instance_process_tram_read(hostmot2_t *hm2,
						   hm2_encoder_t *encoder,
						   int instance) {
    hm2_encoder_instance_t *e = &encoder->instance[instance];

    // sanity check
    if (e->hal.param.scale == 0.0) {
        HM2_ERR("encoder.%02d.scale == 0.0, bogus, setting to 1.0\n", instance);
        e->hal.param.scale = 1.0;
    }

    hm2_encoder_read_control_register(hm2, encoder);

    switch (e->state) {

        case HM2_ENCODER_STOPPED: {
            u16 reg_count;  // the count currently in the register

            // get current count from the FPGA (already read)
            reg_count = hm2_encoder_get_reg_count(encoder, instance);
            if (reg_count == e->prev_reg_count
			&& !(e->prev_control & (HM2_ENCODER_LATCH_ON_INDEX | HM2_ENCODER_LATCH_ON_PROBE))) {
                // still not moving, but .reset can change the position
                hm2_encoder_instance_update_position(hm2, encoder, instance);
                return;
            }

            // if we get here, we *were* stopped but now we're moving
	    // or we are looking for index or probe
            hm2_encoder_instance_update_rawcounts_and_handle_index(hm2, encoder, instance);
            hm2_encoder_instance_update_position(hm2, encoder, instance);

            e->prev_event_rawcounts = *e->hal.pin.rawcounts;
            e->prev_event_reg_timestamp = hm2_encoder_get_reg_timestamp(encoder, instance);
            e->prev_dS_counts = 0;

            e->tsc_num_rollovers = 0;

            e->prev_time_of_interest = e->prev_event_reg_timestamp;

            e->state = HM2_ENCODER_MOVING;

            break;
        }


        case HM2_ENCODER_MOVING: {
            u16 reg_count;  // the count currently in the register
            u16 time_of_interest;  // terrible variable name, sorry

            s32 dT_clocks;
            double dT_s;

            s32 dS_counts;
            double dS_pos_units;

            // get current count from the FPGA (already read)
            reg_count = hm2_encoder_get_reg_count(encoder, instance);
            if (reg_count == e->prev_reg_count) {
                double vel;

                //
                // we're moving, but so slow that we didnt get an event
                // since last time we checked
                // 

                // .reset can still change the position
                hm2_encoder_instance_update_position(hm2, encoder, instance);

                time_of_interest = hm2_encoder_get_reg_tsc(encoder);
                if (time_of_interest < e->prev_time_of_interest) {
                    // tsc rollover!
                    e->tsc_num_rollovers ++;
                }

                dT_clocks = (time_of_interest - e->prev_event_reg_timestamp) + (e->tsc_num_rollovers << 16);
                dT_s = (double)dT_clocks * encoder->seconds_per_tsdiv_clock;

                if (dT_s >= e->hal.param.vel_timeout) {
                    *e->hal.pin.velocity = 0.0;
                    e->state = HM2_ENCODER_STOPPED;
                    break;
                }

                if ((*e->hal.pin.velocity * e->hal.param.scale) > 0.0) {
                    dS_counts = 1;
                } else {
                    dS_counts = -1;
                }

                dS_pos_units = dS_counts / e->hal.param.scale;

                // we can calculate velocity only if timestamp changed along with counts
                if (dT_clocks > 0) {
                    // we know the encoder velocity is not faster than this
                    vel = dS_pos_units / dT_s;
                    if (rtapi_fabs(vel) < rtapi_fabs(*e->hal.pin.velocity)) {
                        *e->hal.pin.velocity = vel;
                    }
                }

		// if waiting for index or latch, we can't shirk our duty just
		// because no pulses arrived
		if(e->prev_control & (HM2_ENCODER_LATCH_ON_INDEX | HM2_ENCODER_LATCH_ON_PROBE)) {
			hm2_encoder_instance_update_rawcounts_and_handle_index(hm2,
									       encoder,
									       instance);
		}

                e->prev_time_of_interest = time_of_interest;

            } else {

                //
                //  we got a new event!
                //

                hm2_encoder_instance_update_rawcounts_and_handle_index(hm2, encoder, instance);
                hm2_encoder_instance_update_position(hm2, encoder, instance);

                time_of_interest = hm2_encoder_get_reg_timestamp(encoder, instance);
                if (time_of_interest < e->prev_time_of_interest) {
                    // tsc rollover!
                    e->tsc_num_rollovers ++;
                }

                dS_counts = *e->hal.pin.rawcounts - e->prev_event_rawcounts;

                // If we reversed direction and moved exactly one edge,
                // we can't reliably estimate velocity.  The encoder might
                // be "balancing on an edge", which may lead to a very
                // small dT between adjacent edges, leading to misleadingly
                // large velocity estimates.  So we just call it 0.
                if (
                    (((*e->hal.pin.rawcounts - e->prev_event_rawcounts) == 1) && (e->prev_dS_counts < 0))
                    || (((*e->hal.pin.rawcounts - e->prev_event_rawcounts) == -1) && (e->prev_dS_counts > 0))
                ) {
                    *e->hal.pin.velocity = 0.0;
                } else {
                    dT_clocks = (time_of_interest - e->prev_event_reg_timestamp) + (e->tsc_num_rollovers << 16);
                    dT_s = (double)dT_clocks * encoder->seconds_per_tsdiv_clock;

                    dS_pos_units = dS_counts / e->hal.param.scale;

                    // we can calculate velocity only if timestamp changed along with counts
                    if (dT_clocks > 0) {
                        // finally time to do Relative-Time Velocity Estimation
                        *e->hal.pin.velocity = dS_pos_units / dT_s;
                    }
                }

                e->tsc_num_rollovers = 0;  // we're "using up" the rollovers now

                e->prev_dS_counts = dS_counts;

                e->prev_event_rawcounts = *e->hal.pin.rawcounts;
                e->prev_event_reg_timestamp = time_of_interest;

                e->prev_time_of_interest = time_of_interest;
            }

            break;
        }


        default: {
            HM2_ERR("encoder %d has invalid state %d, resetting to Stopped!\n", instance, e->state);
            e->state = HM2_ENCODER_STOPPED;
            break;
        }
    }
}




void hm2_encoder_process_tram_read(hostmot2_t *hm2,
				   hm2_encoder_t *encoder,
				   long l_period_ns) {
    int i;

    if (encoder->num_instances <= 0) return;

    // process each encoder instance independently
    for (i = 0; i < encoder->num_instances; i ++) {
        hm2_encoder_instance_process_tram_read(hm2, encoder, i);
    }
}


void hm2_encoder_cleanup(hostmot2_t *hm2, hm2_encoder_t *encoder) {
    if (encoder->num_instances <= 0) return;
    kfree(encoder->control_reg);
}


void hm2_encoder_print_module(hostmot2_t *hm2, hm2_encoder_t *encoder, char *tag) {
    int i;
    HM2_PRINT("%s: %d\n", tag, encoder->num_instances);
    if (encoder->num_instances <= 0) return;
    HM2_PRINT("    clock_frequency: %d Hz (%s MHz)\n", encoder->clock_frequency, hm2_hz_to_mhz(encoder->clock_frequency));
    HM2_PRINT("    version: %d\n", encoder->version);
    HM2_PRINT("    counter_addr: 0x%04X\n", encoder->counter_addr);
    HM2_PRINT("    latch_control_addr: 0x%04X\n", encoder->latch_control_addr);
    HM2_PRINT("    timestamp_div_addr: 0x%04X\n", encoder->timestamp_div_addr);
    HM2_PRINT("    timestamp_count_addr: 0x%04X\n", encoder->timestamp_count_addr);
    HM2_PRINT("    filter_rate_addr: 0x%04X\n", encoder->filter_rate_addr);
    HM2_PRINT("    timestamp_div: 0x%04X\n", (u16)encoder->timestamp_div_reg);
    for (i = 0; i < encoder->num_instances; i ++) {
        HM2_PRINT("    instance %d:\n", i);
        HM2_PRINT("        hw:\n");
        HM2_PRINT("            counter = %04x.%04x\n", (encoder->counter_reg[i] >> 16), (encoder->counter_reg[i] & 0xffff));
        HM2_PRINT("            latch/control = %04x.%04x\n", (encoder->control_reg[i] >> 16), (encoder->control_reg[i] & 0xffff));
        HM2_PRINT("            prev_control = %04x.%04x\n", (encoder->instance[i].prev_control >> 16), (encoder->instance[i].prev_control & 0xffff));
    }
}

