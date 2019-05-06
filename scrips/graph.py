#! /usr/bin/env python3

# external
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import time
import argparse
import os
import subprocess
import atexit
import signal
import psutil
import sys


def signal_recursive(process, sig=signal.SIGTERM):
    try:
        parent = psutil.Process(process.pid)
    except psutil.NoSuchProcess:
        return
    children = parent.children(recursive=True)
    for c in children:
        print("kill process: " + str(c))
        c.send_signal(sig)
        process.send_signal(sig)

def kill_node(node):
    signal_recursive(node)
    node.wait()

def waiton(proc):
    try:
        print('Waiting on proc... ^C to kill it an continue.')
        proc.wait()
    except KeyboardInterrupt:
        signal_recursive(proc, signal.SIGKILL)
        time.sleep(1)
    finally:
        print('Done waiting on proc.')

def launch_node(name):
    return subprocess.Popen(
        '$(catkin_find ltu_actor_graph ' + name + ') ' + ' '.join(sys.argv[1:]),
        shell=True,
        universal_newlines=True,
        stdout=subprocess.PIPE,
        stderr=None,
        start_new_session=True)

if (__name__ == '__main__'):
    argparser = argparse.ArgumentParser()
    argparser.add_argument(
        '-r',
        '--run',
        dest='run',
        type=str,
        required=True,
        help='Automatically start a client')
    args, _ = argparser.parse_known_args()

    sns.set(style='darkgrid')
    try:
        node = launch_node(args.run)
        waiton(node)
        title = node.stdout.readline()
        df = pd.read_csv(node.stdout, sep=',', index_col=0)
        cols = df.columns
        #df['time'] = df.index
        #df = df.melt(value_vars=cols, id_vars=['time'])
        g = sns.lineplot(hue='variable', data=df)
        g.set_title(title)
        plt.show()
    finally:
        kill_node(node)
