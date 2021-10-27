#!/usr/bin/env python3

import argparse
import cv2
import os
from pathlib import Path

def parseArgs():
    argParser = argparse.ArgumentParser()
    argParser.add_argument("--input_path", help="Path to the video.")
    argParser.add_argument("--output", help="Directory to save video frames in.")
    argParser.add_argument("--fps", help="Maximum number of frames to extract per second.")
    argParser.add_argument("--board_width", help="Number of horizontal chessboard squares.")
    argParser.add_argument("--board_height", help="Number of vertical chessboard squares.")
    return argParser.parse_args()

def extractFrames(args):
    board_dimensions = (int(args.board_width), int(args.board_height))

    video = cv2.VideoCapture(args.input_path)
    success, frame = video.read()
    read_count = 0
    write_count = 0
    while success:
        if (chessboardFound(frame, board_dimensions)):
            outputPath = os.path.sep.join([args.output, '{}.jpg'.format(write_count)])
            cv2.imwrite(outputPath, frame)
            write_count += 1

        next_pos = (read_count + 1) * 1000 / int(args.fps)
        video.set(cv2.CAP_PROP_POS_MSEC, next_pos)

        success, frame = video.read()
        read_count += 1

def chessboardFound(frame, board_dimensions):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    success, corners = cv2.findChessboardCorners(gray, board_dimensions, None)
    return success

def makeDir(outputDir):
    path = Path(outputDir)
    path.mkdir(parents=True, exist_ok=True)

if __name__ == '__main__':
    args = parseArgs()
    makeDir(args.output)
    extractFrames(args)
