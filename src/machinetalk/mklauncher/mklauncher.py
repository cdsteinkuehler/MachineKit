#!/usr/bin/python
import os
import sys
import argparse
import zmq
import threading
import time
import signal
import math
import subprocess
import fcntl
import shlex
import dbus

from operator import attrgetter

from machinekit import service
from machinekit import config

from google.protobuf.message import DecodeError
from machinetalk.protobuf.message_pb2 import Container
from machinetalk.protobuf.config_pb2 import *
from machinetalk.protobuf.types_pb2 import *
from machinetalk.protobuf.object_pb2 import ProtocolParameters

if sys.version_info >= (3, 0):
    import configparser
else:
    import ConfigParser as configparser


def printError(msg):
    sys.stderr.write('ERROR: ' + msg + '\n')


class Mklauncher:
    def __init__(self, context, launcherDirs=None, topDir='.',
                 host='', svcUuid=None, debug=False, name=None, hostInName=True,
                 pollInterval=0.5, pingInterval=2.0, loopback=False):
        if launcherDirs is None:
            launcherDirs = []

        self.launcherDirs = launcherDirs
        self.host = host
        self.loopback = loopback
        self.name = name
        self.debug = debug
        self.shutdown = threading.Event()
        self.running = False
        self.pollInterval = pollInterval
        self.pingInterval = pingInterval

        self.container = Container()
        self.txContainer = Container()
        self.launcherSubscribed = False
        self.launcherFullUpdate = False

        self.processes = {}  # for processes mapped to launcher
        self.terminating = set()  # set of terminating processes

        # Create launcher configuration structure
        iniName = 'launcher.ini'
        configDefaults = {
            'name': 'Launcher',
            'command': '',
            'description': '',
            'image': '',
            'shell': 'false',
            'workdir': '.',
            'type': '',
            'manufacturer': '',
            'model': '',
            'variant': '',
            'priority': '0'
        }

        launchers = []
        for rootDir in self.launcherDirs:
            for root, _, files in os.walk(rootDir):
                if iniName in files:
                    iniFile = os.path.join(root, iniName)
                    cfg = configparser.ConfigParser(configDefaults)
                    cfg.read(iniFile)
                    for section in cfg.sections():
                        launcher = Launcher()
                        # descriptive data
                        launcher.name = cfg.get(section, 'name')
                        launcher.description = cfg.get(section, 'description')
                        info = MachineInfo()
                        info.type = cfg.get(section, 'type')
                        info.manufacturer = cfg.get(section, 'manufacturer')
                        info.model = cfg.get(section, 'model')
                        info.variant = cfg.get(section, 'variant')
                        launcher.priority = cfg.getint(section, 'priority')
                        launcher.info.MergeFrom(info)
                        # command data
                        launcher.command = cfg.get(section, 'command')
                        launcher.shell = cfg.getboolean(section, 'shell')
                        workdir = cfg.get(section, 'workdir')
                        if not os.path.isabs(workdir):
                            workdir = os.path.join(root, workdir)
                        launcher.workdir = os.path.normpath(workdir)
                        launcher.returncode = 0
                        launcher.running = False
                        launcher.terminating = False
                        # storing the image file
                        imageFile = cfg.get(section, 'image')
                        if imageFile is not '':
                            if not os.path.isabs(imageFile):
                                imageFile = os.path.join(root, imageFile)
                            fileBuffer = open(imageFile, 'rb').read()
                            image = File()
                            image.name = os.path.basename(imageFile)
                            image.encoding = CLEARTEXT
                            image.blob = fileBuffer
                            launcher.image.MergeFrom(image)
                        launchers.append(launcher)

        # sort using the priority attribute before distribution
        launchers = sorted(launchers, key=attrgetter('priority'), reverse=True)
        for index, launcher in enumerate(launchers):
            launcher.index = index
            self.container.launcher.add().CopyFrom(launcher)
            self.txContainer.launcher.add().MergeFrom(launcher)

        if self.debug:
            print(self.container)

        # prepare pings
        if self.pingInterval > 0:
            self.pingRatio = math.floor(self.pingInterval / self.pollInterval)
        else:
            self.pingRatio = -1
        self.pingCount = 0

        self.rx = Container()
        self.txCommand = Container()
        self.topDir = topDir
        self.context = context
        self.baseUri = "tcp://"
        if self.loopback:
            self.baseUri += '127.0.0.1'
        else:
            self.baseUri += '*'
        self.launcherSocket = context.socket(zmq.XPUB)
        self.launcherSocket.setsockopt(zmq.XPUB_VERBOSE, 1)
        self.launcherPort = self.launcherSocket.bind_to_random_port(self.baseUri)
        self.launcherDsname = self.launcherSocket.get_string(zmq.LAST_ENDPOINT, encoding='utf-8')
        self.launcherDsname = self.launcherDsname.replace('0.0.0.0', self.host)
        self.commandSocket = context.socket(zmq.ROUTER)
        self.commandPort = self.commandSocket.bind_to_random_port(self.baseUri)
        self.commandDsname = self.commandSocket.get_string(zmq.LAST_ENDPOINT, encoding='utf-8')
        self.commandDsname = self.commandDsname.replace('0.0.0.0', self.host)

        if self.name is None:
            self.name = 'Machinekit Launcher'
        if hostInName:
            self.name += ' on ' + self.host
        self.launcherService = service.Service(type='launcher',
                                   svcUuid=svcUuid,
                                   dsn=self.launcherDsname,
                                   port=self.launcherPort,
                                   host=self.host,
                                   name=self.name,
                                   loopback=self.loopback,
                                   debug=self.debug)
        self.commandService = service.Service(type='launchercmd',
                                   svcUuid=svcUuid,
                                   dsn=self.commandDsname,
                                   port=self.commandPort,
                                   host=self.host,
                                   loopback=self.loopback,
                                   debug=self.debug)

        self.publish()

        threading.Thread(target=self.process_sockets).start()
        threading.Thread(target=self.poll).start()
        self.running = True

    def process_sockets(self):
        poll = zmq.Poller()
        poll.register(self.launcherSocket, zmq.POLLIN)
        poll.register(self.commandSocket, zmq.POLLIN)

        while not self.shutdown.is_set():
            s = dict(poll.poll(1000))
            if self.launcherSocket in s and s[self.launcherSocket] == zmq.POLLIN:
                self.process_launcher(self.launcherSocket)
            if self.commandSocket in s and s[self.commandSocket] == zmq.POLLIN:
                self.process_command(self.commandSocket)

        self.unpublish()
        self.running = False
        return

    def publish(self):
        # Zeroconf
        try:
            self.launcherService.publish()
            self.commandService.publish()
        except Exception as e:
            print (('cannot register DNS service' + str(e)))
            sys.exit(1)

    def unpublish(self):
        self.launcherService.unpublish()
        self.commandService.unpublish()

    def stop(self):
        self.terminate_processes()
        self.shutdown.set()

    def add_pparams(self):
        parameters = ProtocolParameters()
        parameters.keepalive_timer = int(self.pingInterval * 1000.0)
        self.txContainer.pparams.MergeFrom(parameters)

    def update_launcher(self):
        modified = False
        for launcher in self.container.launcher:
            index = launcher.index

            terminating = False
            if index in self.terminating:
                terminating = True
                self.terminating.remove(index)

            if index in self.processes:
                txLauncher = Launcher()  # new pb message for tx
                txLauncher.index = index
                process = self.processes[index]
                process.poll()
                returncode = process.returncode
                if returncode is None:
                    if not launcher.running:  # update running value
                        if len(launcher.output) > 0:
                            launcher.ClearField('output')  # clear output for new processes
                            self.launcherFullUpdate = True  # request a full update
                        txLauncher.running = True
                        txLauncher.returncode = 0
                        modified = True
                    # read stdout
                    stdoutIndex = len(launcher.output)
                    while True:
                        try:
                            line = process.stdout.readline()
                            stdoutLine = StdoutLine()
                            stdoutLine.index = stdoutIndex
                            stdoutLine.line = line
                            txLauncher.output.add().MergeFrom(stdoutLine)
                            stdoutIndex += 1
                            modified = True
                        except IOError:  # process has no new line
                            break
                    # send termination status
                    if terminating:
                        txLauncher.terminating = True
                else:
                    txLauncher.returncode = returncode
                    txLauncher.running = False
                    txLauncher.terminating = False
                    modified = True
                    self.processes.pop(index, None)  # remove from watchlist
                if modified:
                    launcher.MergeFrom(txLauncher)
                    self.txContainer.launcher.add().MergeFrom(txLauncher)

        if self.launcherFullUpdate:
            self.add_pparams()
            self.txContainer.CopyFrom(self.container)
            self.send_launcher_msg(MT_LAUNCHER_FULL_UPDATE)
            self.launcherFullUpdate = False
        elif modified:
            self.send_launcher_msg(MT_LAUNCHER_INCREMENTAL_UPDATE)

    def send_launcher_msg(self, msgType):
        if self.debug:
            print('sending launcher message')
        self.txContainer.type = msgType
        txBuffer = self.txContainer.SerializeToString()
        self.txContainer.Clear()
        self.launcherSocket.send_multipart(['launcher', txBuffer], zmq.NOBLOCK)

    def send_command_msg(self, identity, msgType):
        self.txCommand.type = msgType
        txBuffer = self.txCommand.SerializeToString()
        self.commandSocket.send_multipart(identity + [txBuffer], zmq.NOBLOCK)
        self.txCommand.Clear()

    def poll(self):
        while not self.shutdown.is_set():
            if self.launcherSubscribed:
                self.update_launcher()
                if (self.pingCount == self.pingRatio):
                    self.ping_launcher()

            if (self.pingCount == self.pingRatio):
                self.pingCount = 0
            else:
                self.pingCount += 1
            time.sleep(self.pollInterval)

        self.running = False
        return

    def ping_launcher(self):
        self.send_launcher_msg(MT_PING)

    def process_launcher(self, s):
        try:
            rc = s.recv()
            subscription = rc[1:]
            status = (rc[0] == "\x01")

            if subscription == 'launcher':
                self.launcherSubscribed = status
                self.launcherFullUpdate = status

            if self.debug:
                print(("process launcher called " + subscription + ' ' + str(status)))

        except zmq.ZMQError as e:
            printError('ZMQ error: ' + str(e))

    def start_process(self, index):
        launcher = self.container.launcher[index]
        workdir = launcher.workdir
        shell = launcher.shell
        command = launcher.command
        if shell is False:
            command = shlex.split(command)
        try:
            process = subprocess.Popen(command,
                                       shell=shell,
                                       cwd=workdir,
                                       stdout=subprocess.PIPE,
                                       stderr=subprocess.STDOUT,
                                       stdin=subprocess.PIPE,
                                       preexec_fn=os.setsid)
        except OSError as e:
            return False, str(e)
        process.command = command
        # set the O_NONBLOCK flag of stdout file descriptor:
        flags = fcntl.fcntl(process.stdout, fcntl.F_GETFL)  # get current stdout flags
        fcntl.fcntl(process.stdout, fcntl.F_SETFL, flags | os.O_NONBLOCK)
        self.processes[index] = process
        return True, ''

    def terminate_process(self, index):
        pid = self.processes[index].pid
        os.killpg(pid, signal.SIGTERM)
        self.terminating.add(index)

    def kill_process(self, index):
        pid = self.processes[index].pid
        os.killpg(pid, signal.SIGKILL)
        self.terminating.add(index)

    def terminate_processes(self):
        for index in self.processes.keys():
            self.terminate_process(index)

    def write_stdin_process(self, index, data):
        self.processes[index].stdin.write(data)

    def shutdown_system(self):
        try:
            systemBus = dbus.SystemBus()
            ckService = systemBus.get_object('org.freedesktop.ConsoleKit',
                                             '/org/freedesktop/ConsoleKit/Manager')
            ckInterface = dbus.Interface(ckService, 'org.freedesktop.ConsoleKit.Manager')
            stopMethod = ckInterface.get_dbus_method("Stop")
            stopMethod()
            return True
        except:
            return False

    def send_command_wrong_params(self, identity, note='wrong parameters'):
        self.txCommand.note.append(note)
        self.send_command_msg(identity, MT_ERROR)

    def send_command_wrong_index(self, identity):
        self.txCommand.note.append('wrong index')
        self.send_command_msg(identity, MT_ERROR)

    def process_command(self, s):
        frames = s.recv_multipart()
        identity = frames[:-1]  # multipart id
        message = frames[-1]  # last frame

        if self.debug:
            print("process command called, id: %s" % identity)

        try:
            self.rx.ParseFromString(message)
        except DecodeError as e:
            note = 'Protobuf Decode Error: ' + str(e)
            self.send_command_wrong_params(identity, note=note)
            return

        if self.rx.type == MT_PING:
            self.send_command_msg(identity, MT_PING_ACKNOWLEDGE)

        elif self.rx.type == MT_LAUNCHER_START:
            if self.rx.HasField('index'):
                index = self.rx.index
                if index >= len(self.container.launcher):
                    self.send_command_wrong_index(identity)
                else:
                    success, note = self.start_process(index)
                    if not success:
                        self.txCommand.note.append(note)
                        self.send_command_msg(identity, MT_ERROR)
            else:
                self.send_command_wrong_params(identity)

        elif self.rx.type == MT_LAUNCHER_TERMINATE:
            if self.rx.HasField('index'):
                index = self.rx.index
                if index >= len(self.container.launcher) \
                   or index not in self.processes:
                    self.send_command_wrong_index(identity)
                else:
                    self.terminate_process(index)

        elif self.rx.type == MT_LAUNCHER_KILL:
            if self.rx.HasField('index'):
                index = self.rx.index
                if index >= len(self.container.launcher) \
                   or index not in self.processes:
                    self.send_command_wrong_index(identity)
                else:
                    self.kill_process(index)

        elif self.rx.type == MT_LAUNCHER_WRITE_STDIN:
            if self.rx.HasField('index') \
               and self.rx.HasField('name'):  # temporarily using the name field
                index = self.rx.index
                name = self.rx.name
                if index >= len(self.container.launcher) \
                   or index not in self.processes:
                    self.send_command_wrong_index(identity)
                else:
                    self.write_stdin_process(index, name)

        elif self.rx.type == MT_LAUNCHER_CALL:
            self.txCommand.note.append("process call not allowed")
            self.send_command_msg(identity, MT_ERROR)

        elif self.rx.type == MT_LAUNCHER_SHUTDOWN:
            if not self.shutdown_system():
                self.txCommand.note.append("cannot shutdown system: DBus error")
                self.send_command_msg(identity, MT_ERROR)

        else:
            self.txCommand.note.append("unknown command")
            self.send_command_msg(identity, MT_ERROR)


shutdown = False


def _exitHandler(signum, frame):
    del signum  # ignored
    del frame  # ignored
    global shutdown
    shutdown = True


# register exit signal handlers
def register_exit_handler():
    signal.signal(signal.SIGINT, _exitHandler)
    signal.signal(signal.SIGTERM, _exitHandler)


def check_exit():
    global shutdown
    return shutdown


def main():
    parser = argparse.ArgumentParser(description='mklauncher is Machinetalk based session/configuration launcher for Machinekit')
    parser.add_argument('-n', '--name', help='Name of the machine', default="Machinekit Launcher")
    parser.add_argument('-s', '--suppress_ip', help='Do not show ip of machine in service name', action='store_false')
    parser.add_argument('-d', '--debug', help='Enable debug mode', action='store_true')
    parser.add_argument('dirs', nargs='*', help="List of directories to scan for launcher configurations")

    args = parser.parse_args()
    debug = args.debug

    mkconfig = config.Config()
    mkini = os.getenv("MACHINEKIT_INI")
    if mkini is None:
        mkini = mkconfig.MACHINEKIT_INI
    if not os.path.isfile(mkini):
        sys.stderr.write("MACHINEKIT_INI " + mkini + " does not exist\n")
        sys.exit(1)

    mki = configparser.ConfigParser()
    mki.read(mkini)
    uuid = mki.get("MACHINEKIT", "MKUUID")
    remote = mki.getint("MACHINEKIT", "REMOTE")

    if remote == 0:
        print("Remote communication is deactivated, configserver will use the loopback interfaces")
        print(("set REMOTE in " + mkini + " to 1 to enable remote communication"))

    if debug:
        print(("announcing mklauncher"))

    context = zmq.Context()
    context.linger = 0

    register_exit_handler()

    hostname = '%(fqdn)s'  # replaced by service announcement
    mklauncher = Mklauncher(context,
                            svcUuid=uuid,
                            topDir='.',
                            host=hostname,
                            launcherDirs=args.dirs,
                            name=args.name,
                            hostInName=bool(args.suppress_ip),
                            loopback=(not remote),
                            debug=debug)

    while mklauncher.running and not check_exit():
        time.sleep(1)

    if debug:
        print('stopping threads')
    if mklauncher is not None:
        mklauncher.stop()

    # wait for all threads to terminate
    while threading.active_count() > 1:
        time.sleep(0.1)

    if debug:
        print('threads stopped')
    sys.exit(0)

if __name__ == "__main__":
    main()
