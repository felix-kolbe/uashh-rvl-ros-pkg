'''
Created on Dec 11, 2013

@author: felix
'''

import logging

import rgoap


def test():
    logger = logging.getLogger('rgoap')

    logger.error("msg..logging.error (%s)", "arg")
    logger.info("msg..logging.info (%s)", "arg")



if __name__ == '__main__':
    test()
