#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Mar 16 13:40:05 2019

Usage:
    remove_non_user_records.py [--tub=<tub>]

Options:
    -h --help        Show this screen.

"""

import os
import json

from docopt import docopt

if __name__ == '__main__':
    args = docopt(__doc__)
    tub_path=args['--tub']
    
    files = os.listdir(tub_path)
    record_files = [f for f in files if f[:6] == 'record']
    
    def get_file_ix(file_name):
        try:
            name = file_name.split('.')[0]
            num = int(name.split('_')[1])
        except:
            num = 0
        return num
    
    count = 0
    for f in record_files:
        path_name = os.path.join(tub_path,f)
        with open(path_name, 'r') as jason_file:
            data = json.load(jason_file)
        if data["user/mode"] != "user":
            #delete jason and image
            os.remove(path_name)
            #remove jpg
            jpg = str(get_file_ix(f)) + '_cam-image_array_.jpg'
            try:
                os.remove(os.path.join(tub_path,jpg))
            except OSError:
                pass
            count += 1
    print("Deleted " + str(count) + " entries")    
            

