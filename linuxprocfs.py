#!/usr/bin/env python
#
# Copyright (C) 2011: Michael Hamilton
# The code is LGPL (GNU Lesser General Public License) ( http://www.gnu.org/copyleft/lesser.html )
#

from __future__ import with_statement
import re
import os
import glob
import string
import pwd
import csv
import sys
import time
from optparse import OptionParser
import signal
import sys

PROC_FS_ROOT = '/proc'
INT_RE_SPEC = '[+-]*\d+'
INT_RE = re.compile(INT_RE_SPEC + '$')
CSV_LINE_TERMINATOR='\n'


# Default parser that deals with a multi-line file where each line 
# is a "tag: value" pair 
class _ProcBase(object):

    _split_re = re.compile(':\s+')

    def __init__(self, path=None, filename=None):
        self.error = None
        if path and filename:
            self.parseProcFs(path, filename)

    def parseProcFs(self, path, filename):
        pid = os.path.basename(path)
        self.pid = int(pid) if INT_RE.match(pid) else pid
        try:
            with open(path + '/' + filename) as proc_file:
                for line in proc_file.read().splitlines():
                    sort_key, value = _ProcBase._split_re.split(line)
                    self.__dict__[string.lower(sort_key)] = int(value) if INT_RE.match(value) else value
        except IOError as ioerr:
            self.handle_error('IOError %s/%s - %s' % (path, filename, ioerr))

    def handle_error(self, message):
        self.error = message
        print >> sys.stderr, self.error

    def keys(self):
        return sorted(self.__dict__.keys())

    def csv(self, file, header=True):
        if not self.error:
            if header:
                csv.writer(file, lineterminator=CSV_LINE_TERMINATOR).writerow(self.keys())
            csv.DictWriter(sys.stdout,  self.keys(), lineterminator=CSV_LINE_TERMINATOR).writerow(self.__dict__)

# Parser for space separated Values on one line- e.g. "12 comm 123456 111 a 12"
class _SpaceSeparatedParser(object):

    def __init__(self):
        self._keys = []
        self._re_spec = ''
        self._regexp = None

    def _add_item(self, sort_key, rexp_str):
        self._regexp = None
        self._keys.append(sort_key)
        if rexp_str:
            self._re_spec += rexp_str % sort_key
        return self
    def int_item(self, sort_key):
        return self._add_item(sort_key, '(?P<%s>' + INT_RE_SPEC + ')\s')
    def comm_item(self, sort_key):
        return self._add_item(sort_key, '[(](?P<%s>[^)]+)[)]\s')
    def string_item(self, sort_key):
        return self._add_item(sort_key, '(?P<%s>\w+)\s')
    def nonparsed_item(self, sort_key):  # Create property sort_key only, but don't parse it
        return self._add_item(sort_key, None)

    def keys(self):
        return self._keys;
    def parse(self, line):
        if not self._regexp:
            self._regexp = re.compile(self._re_spec)
        return self._regexp.match(line)

class ProcStat(_ProcBase):

    _parser = _SpaceSeparatedParser().\
        int_item('pid').\
        comm_item('comm').\
        string_item('state').\
        int_item('ppid').\
        int_item('pgrp').\
        int_item('session').\
        int_item('tty_nr').\
        int_item('tpgid').\
        int_item('flags').\
        int_item('minflt').\
        int_item('cminflt').\
        int_item('majflt').\
        int_item('cmajflt').\
        int_item('utime').\
        int_item('stime').\
        int_item('cutime').\
        int_item('cstime').\
        int_item('priority').\
        int_item('nice').\
        int_item('num_threads').\
        int_item('itrealvalue').\
        int_item('starttime').\
        int_item('vsize').\
        int_item('rss').\
        int_item('rlim').\
        int_item('startcode').\
        int_item('endcode').\
        int_item('startstack').\
        int_item('kstkesp').\
        int_item('kstkeip').\
        int_item('signal').\
        int_item('blocked').\
        int_item('sigignore').\
        int_item('sigcatch').\
        int_item('wchan').\
        int_item('nswap').\
        int_item('cnswap').\
        int_item('exit_signal').\
        int_item('processor').\
        int_item('rt_priority').\
        int_item('policy').\
        int_item('delayacct_blkio_ticks').\
        int_item('guest_time').\
        int_item('cguest_time').\
        nonparsed_item('error')

    def __init__(self, path):
        _ProcBase.__init__(self)
        if path:
            self.parseProcFs(path)

    def parseProcFs(self, path):
        path = path + '/stat'
        try:
            with open(path) as stat_file:
                for line in stat_file: # Only one line in file
                    if line and line != '':
                        self.parse(line)
                        self.error = None
                    else:
                        self.error = 'Empty line'
        except IOError as ioerr:
            self.handle_error('IOError %s - %s' % (path, ioerr))


    def parse(self, line):
        # Dynamically (at run time) add properties to this instance representing
        # each stat value.  E.g. add the pid value as a field called self.pid
        split_line = ProcStat._parser.parse(line);
        if split_line:
            # Update the properties of the Stat instance with integer or
            # string values as appropriate.
            for sort_key, value in split_line.groupdict().items():
                self.__dict__[sort_key] = int(value) if INT_RE.match(value) else value
        else:
            self.handle_error('Failed to match:' + line)

    def keys(self):
        return ProcStat._parser.keys()



class ProcStatus(_ProcBase):

    def __init__(self, path):
        _ProcBase.__init__(self, path, 'status')
        if not self.error:
            self.uid = [ int(uid) for uid in string.split(self.uid,'\t')]

class ProcIO(_ProcBase):

    def __init__(self, path):
        _ProcBase.__init__(self, path, 'io')


class ProcInfo(object):

    def __init__(self, path):
        self.time_stamp = time.time()
        self.meta = {}
        self.stat = ProcStat(path)
        self.status = ProcStatus(path)
        self.io = ProcIO(path)
        self.username = pwd.getpwuid(self.status.uid[0]).pw_name if not self.hasErrors() else 'nobody'
        self.pid = int(path.split('/')[-1])

    def hasErrors(self):
        return self.stat.error or self.status.error or self.io.error

def get_all_proc_data(include_threads=False, root=PROC_FS_ROOT):
    if include_threads:
        results = [ProcInfo(task_path) for task_path in glob.glob(root + '/[0-9]*/task/[0-9]*')]
    else:
        results = [ProcInfo(task_path) for task_path in glob.glob(root + '/[0-9]*')]
    return [info for info in results if not info.hasErrors()]

def get_proc_info(pid, threadid=None, root=PROC_FS_ROOT):
    return ProcInfo(root + '/' + pid + ('task/' + threadid) if threadid else '')
def get_proc_stat(pid, threadid=None, root=PROC_FS_ROOT):
    return ProcStat(root + '/' + pid + ('task/' + threadid) if threadid else '')
def get_proc_status(pid, threadid=None, root=PROC_FS_ROOT):
    return ProcStatus(root + '/' + pid + ('task/' + threadid) if threadid else '')
def get_proc_io(pid, threadid=None, root=PROC_FS_ROOT):
    return ProcIO(root + '/' + pid + ('task/' + threadid) if threadid else '')


def signal_handler(sig, frame):
    global running
    running=False

if __name__ == '__main__':
    global running
    running = True
    signal.signal(signal.SIGINT, signal_handler)
    usage = """usage: %prog [options] [pid...]
    Output CSV for procfs stat, status or io data for given thread/process pid's or
    for all processes and threads if no pid's are supplied."""
    parser = OptionParser(usage)
    parser.add_option('-s', '--stat', action='store_true', dest='do_stat', help='Output csv for pid/stat files.')
    parser.add_option('-S', '--status', action='store_true', dest='do_status', help='Output csv for pid/status files.')
    parser.add_option('-i', '--io', action='store_true', dest='do_io', help='Output csv for pid/io files.')
    parser.add_option('-t', '--titles', action='store_true', dest='output_titles', help='Output a title line.')
    parser.add_option('-r', '--repeat', action='store_true', dest='repeat', help='Repeat until interrupted.')
    parser.add_option('-w', '--sleep', type='int', dest='wait', default=5, help='Sleep seconds for each repetition.')
    parser.add_option('-p', '--processes', action='store_true', dest='processes_only', help='Show all processes, but not threads.')

    (options, args) = parser.parse_args()
    header = options.output_titles

    if len(args) == 0:
        args = [ '[0-9]*' ] # match all processes or threads
    elif options.processes_only:
        print >> sys.stderr, 'ignoring -p, showing requested processes and threads instead.'
        options.processes_only = False

    while running:
        for pid in args:
            for path in glob.glob(PROC_FS_ROOT + ('/' if options.processes_only else '/[0-9]*/task/') + pid):
                if options.do_stat or (not options.do_status and not options.do_io):
                    ProcStat(path).csv(sys.stdout, header=header)
                if options.do_status:
                    ProcStatus(path).csv(sys.stdout, header=header)
                if options.do_io:
                    ProcIO(path).csv(sys.stdout, header=header)
                header = False
                sys.stdout.flush()
        if not options.repeat:
            break
        if not running:
            break
        time.sleep(options.wait)
    sys.stdout.flush()
