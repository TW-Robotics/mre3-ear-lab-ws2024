#!/usr/bin/env python3

from operator import le
import os
import sys
import getopt
import time
import datetime
import locale
import shutil
import tarfile
from logging import Logger, INFO, DEBUG, WARN
import rospkg
import paramiko
from scp import SCPClient


def progress(filename, size, sent):
    sys.stdout.write(
        "%s's progress: %.2f%%   \r" % (filename, float(sent) / float(size) * 100)
    )


def progress4(filename, size, sent, peername):
    sys.stdout.write(
        "(%s:%s) %s's progress: %.2f%%   \r"
        % (peername[0], peername[1], filename, float(sent) / float(size) * 100)
    )


class AutoBackup:
    """_summary_"""

    def __init__(self, looptime: int, scp: bool):
        self.logger = Logger("AutoBackup", level=INFO)
        self.starttime = time.time()
        self.looptime = looptime
        self.scp = scp
        self.date = datetime.datetime.now()
        self.directory = None
        locale.setlocale(locale.LC_ALL, "de_DE.UTF-8")
        self.rospack = rospkg.RosPack()

        if os.path.isdir("./backup") is False:
            os.mkdir("./backup")

        self.ssh = self.createSSHClient()
        self.logger.info("SSH connection established")
        # self.scp = SCPClient(self.ssh.get_transport(), progress=self.progress)
        # self.scp = SCPClient(self.ssh.get_transport(), progress4=self.progress4)

    def save_map(self):
        """Saves the current 2D map"""
        string = "rosrun map_server map_saver -f " + self.directory + "/map"
        os.system(string)

    def save_3d_map(self):
        """Saves the current octomap"""
        string = (
            "rosrun octomap_server octomap_saver -f " + self.directory + "/3d_map.ot"
        )
        os.system(string)

    def copy_csv(self):
        """Creates a local copy of the detected objects and rn measurements"""
        dest = "./backup/" + self.date.strftime("%Y%m%d_%H%M%S")
        src_sign = self.rospack.get_path("detection") + "/human.csv"
        src_human = self.rospack.get_path("detection") + "/sign.csv"
        src_gcount = self.rospack.get_path("gcount") + "/gcounts_neu.csv"
        self.logger.debug("path dest= %s", dest)
        self.logger.debug("path src_sign= %s", src_sign)
        self.logger.debug("path src_human= %s", src_human)
        self.logger.debug("path src_gcount= %s", src_gcount)

        if os.path.isfile(src_sign) is True:
            shutil.copy2(src_sign, dest)
        if os.path.isfile(src_human) is True:
            shutil.copy2(src_human, dest)
        if os.path.isfile(src_gcount) is True:
            shutil.copy2(src_gcount, dest)

    def safe_to_operator(self):
        """Scp the tar file to operator laptop"""
        src = self.directory + ".tar.gz"
        dest = (
            "/home/elrob/tracker_backup/"
            + self.date.strftime("%Y%m%d_%H%M%S")
            + ".tar.gz"
        )

        with SCPClient(self.ssh.get_transport(), progress4=progress4) as scp_:
            scp_.put(src, dest)

    def loop(self):
        """Run function"""
        while True:
            self.logger.info("Autosave activated")

            self.date = datetime.datetime.now()
            self.directory = "./backup/" + self.date.strftime("%Y%m%d_%H%M%S")
            tar = self.directory + ".tar.gz"
            os.mkdir(self.directory)

            self.save_map()
            self.save_3d_map()
            self.copy_csv()

            with tarfile.open(tar, "w:gz") as tar:
                tar.add(self.directory, arcname=os.path.basename(self.directory))

            if self.scp is True:
                self.safe_to_operator()

            self.logger.info(
                "Autosave finished - resting for %i seconds", self.looptime
            )

            self.directory = None
            time.sleep(self.looptime - ((time.time() - self.starttime) % self.looptime))

    def createSSHClient(self):
        """Establishes the ssh/scp connection"""
        # ToDo: Exchange for static IP and username
        key = paramiko.RSAKey.from_private_key_file(
            "/home/workstation/.ssh/tau_to_operator", password="&TauRob&"
        )
        proxy = None
        client = paramiko.SSHClient()
        ssh_config_file = os.path.expanduser("~/.ssh/config")
        if os.path.isfile(ssh_config_file):
            conf = paramiko.SSHConfig()
            with open(ssh_config_file) as f:
                conf.parse(f)
            host_config = conf.lookup("operator")
            if "proxycommand" in host_config:
                proxy = paramiko.ProxyCommand(host_config["proxycommand"])

        client.load_system_host_keys()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

        client.connect(
            hostname=host_config.get("hostname"),
            username=host_config.get("user"),
            pkey=key,
            sock=proxy,
        )
        return client

    def shutdown(self):
        """Save everything one more time"""
        self.date = datetime.datetime.now()
        self.directory = "./backup/" + self.date.strftime("%Y%m%d_%H%M%S")
        tar = self.directory + ".tar.gz"
        os.mkdir(self.directory)

        self.save_map()
        self.save_3d_map()
        self.copy_csv()

        with tarfile.open(tar, "w:gz") as tar:
            tar.add(self.directory, arcname=os.path.basename(self.directory))

        self.safe_to_operator()
        self.logger.info("Autosave finished - shutting down")


if __name__ == "__main__":
    ab = None
    try:
        looptime = 60
        scp = True

        try:
            opts, args = getopt.getopt(
                sys.argv[1:], "h:l:s:", ["looptime=", "scptransfer="]
            )
        except getopt.GetoptError:
            print("automatic_map_csv_saver.py -l <looptime> -s <scptransfer>")
            sys.exit(2)

        for opt, arg in opts:
            if opt == "-h":
                print("automatic_map_csv_saver.py -l <looptime> -s <scptransfer>")
                sys.exit()
            elif opt in ("-l", "--looptime"):
                if int(arg) < 1:
                    print("looptime has to be greater 0")
                    sys.exit(2)
                else:
                    looptime = int(arg)
            elif opt in ("-s", "--scptransfer"):
                if int(arg) == 0:
                    scp = False
                elif int(arg) == 1:
                    scp = True
                else:
                    print("scptranfer has to be 0 or 1")
                    sys.exit(2)

        print("[ INFO]: Started auto_backup of map and csv")
        ab = AutoBackup(looptime, scp)
        ab.loop()
    except (KeyboardInterrupt, SystemExit):
        ab.shutdown()
        print("[ INFO]: Stopped auto_backup of map and csv")
