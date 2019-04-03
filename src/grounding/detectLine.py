#!/usr/bin/env python


import cv2
import numpy as np
import os

import rospy
from topology_map.msg import *

searchR = 50
stepC = 100
stepSize = np.pi / stepC * 2

lineDevideC = 50

isD = False
isShowingResult = True

# 1: moving on the edge
# 2: moving to node
# 3: aimming at node
# 4: leaving from node
mode = 4
outDir = [1, 0]
aimCount = 0

path = 'pics/'


def getiDevided(i, twoP):
    """
    fuck you
    :param i: asdf
    :param twoP: asdf
    :return:
    """
    x = twoP[0][0] * i + twoP[1][0] * (1 - i)
    y = twoP[0][1] * i + twoP[1][1] * (1 - i)
    return [int(x), int(y)]


def exractTopoStructor(img2v, x0, y0):
    thBegin = 0
    for i in range(0, stepC):
        th = i * stepSize
        rx = int(searchR * np.cos(th) + x0)
        ry = int(searchR * np.sin(th) + y0)
        if img2v[ry][rx] == 0:
            thBegin = th
            break

    ringPairs = []
    ringDir = []
    isInRing = False
    for i in range(1, stepC + 1):
        th = thBegin + i * stepSize
        rx = int(searchR * np.cos(th) + x0)
        ry = int(searchR * np.sin(th) + y0)
        if isInRing:
            if img2v[ry][rx] == 0:
                ringPairs[-1].append(th)
                isInRing = False
        else:
            if img2v[ry][rx] == 255:
                ringPairs.append([th])
                isInRing = True

    if len(ringPairs) != 0:
        if len(ringPairs[-1]) == 1:
            ringPairs[-1].append(thBegin + 2 * np.pi)

        if ringPairs[-1][1] < ringPairs[0][1]:
            ringPairs[-1][1] += 2 * np.pi

        for ringPair in ringPairs:
            th = (ringPair[0] + ringPair[1]) / 2
            ringDir.append(th)

    return ringDir


if __name__ == "__main__":

    rospy.init_node('lineDetecter')

    pub_imageInfo = rospy.Publisher("topo/cvInfo", ImageExract, queue_size = 0)

    rate = rospy.Rate(10)

    cap = cv2.VideoCapture(0)

    while not rospy.is_shutdown():

        imageExtract = ImageExract()
        imageExtract.nodePosX = -1
        imageExtract.nodePosY = -1
        imageExtract.exitDirs = []

        # image processing
        ret, imgori = cap.read()
        sizex = imgori.shape[1]
        sizey = imgori.shape[0]

        imageExtract.imageSizeX = sizex
        imageExtract.imageSizeY = sizey

        lowbd = np.array([110, 175, 110])
        highbd = np.array([255, 255, 255])

        # turn it into 2 value
        img2v = cv2.inRange(imgori, lowbd, highbd)

        if isD:
            cv2.imshow('im', imgori)
            cv2.waitKey()
            cv2.imshow('im', img2v)
            cv2.waitKey()
            cv2.imshow('im', imgori)
            cv2.waitKey()

        ke = np.ones((2, 2))
        img2v = cv2.erode(img2v, ke, iterations=3)
        kd = np.ones((5, 5))
        img2v = cv2.dilate(img2v, kd)

        if isD:
            cv2.imshow('im', img2v)
            cv2.waitKey()

        if isShowingResult:
            cv2.imshow('2vMap', img2v)
            cv2.waitKey(1)

        # detect the edge
        imgLine = cv2.Canny(img2v, 5, 200, 3)
        if isD:
            cv2.imshow('im', imgLine)
            cv2.waitKey()

        lines = cv2.HoughLines(imgLine, 1, np.pi / 180, 50)

        if lines is not None:
            # { theta : [[thetas], [rhs]] }
            lineInfos = {}
            lines1 = lines[:, 0, :]
            for pair in lines1[:]:

                rho = pair[0]
                theta = pair[1]

                dirHasRecorded = False
                for angle in lineInfos.keys():

                    # if the th is negative with the angle, put it back and negative the rh
                    if abs(abs(angle - theta) - np.pi) < 0.3:
                        theta = angle
                        rho = -rho

                    if abs(angle - theta) < 0.3:
                        lineInfos[angle][0].append(theta)
                        lineInfos[angle][1].append(rho)
                        dirHasRecorded = True  # it's not a new dir
                        break

                if dirHasRecorded:
                    continue

                lineInfos[theta] = [[theta], [rho]]

            linesExracted = []
            for lineSet in lineInfos.values():
                # we just average it, which looks good
                th = np.average(lineSet[0])
                rh = np.average(lineSet[1])
                linesExracted.append([th, rh])

            # one line situation
            if len(linesExracted) == 1:

                a1, r1 = linesExracted[0]

                imageExtract.rhs.append(r1)
                imageExtract.ths.append(a1)

                # jie ju
                if a1 == 0:
                    a1 = 0.00001
                x1 = r1 / np.cos(a1)
                y1 = r1 / np.sin(a1)
                x2 = x1 - sizey * np.tan(a1)
                y2 = y1 - sizex / np.tan(a1)

                edgePoints = []

                if 0 < x1 <= sizex:
                    edgePoints.append([int(x1), 0])

                if 0 < x2 <= sizex:
                    edgePoints.append([int(x2), sizey])

                if 0 <= y1 < sizey:
                    edgePoints.append([0, int(y1)])

                if 0 < y2 <= sizey:
                    edgePoints.append([sizex, int(y2)])

                xb, yb = getiDevided(0.25, edgePoints)
                xe, ye = getiDevided(0.75, edgePoints)
                bIsLine = img2v[yb][xb] == 255
                eIsLine = img2v[ye][xe] == 255

                # this is just a line
                if bIsLine and eIsLine:
                    midx = (edgePoints[0][0] + edgePoints[1][0]) / 2
                    midy = (edgePoints[0][1] + edgePoints[1][1]) / 2

                    if isShowingResult:
                        cv2.circle(imgori, (midx, midy), 10, (0, 0, 255), 5)
                        cv2.imshow('res', imgori)
                        cv2.waitKey(1)

                # it's a node
                else:
                    if bIsLine:
                        beginType = 255
                    else:
                        beginType = 0

                    x = xb
                    y = yb
                    for i in range(lineDevideC / 4, lineDevideC * 3 / 4):
                        x, y = getiDevided(float(i) / lineDevideC, edgePoints)
                        if img2v[y][x] != beginType:
                            break

                    imageExtract.nodePosX = x
                    imageExtract.nodePosY = y
                    if bIsLine:
                        direction = np.arctan2(yb - y, xb - x)
                    else:
                        direction = np.arctan2(ye - y, xe - x)
                    imageExtract.exitDirs.append(direction)

                    if isShowingResult:
                        cv2.circle(imgori, (x, y), 10, (0, 0, 255), 5)

                        if bIsLine:
                            cv2.line(imgori, (x, y), (xb, yb), (255, 0, 0), 5)
                        else:
                            cv2.line(imgori, (x, y), (xe, ye), (255, 0, 0), 5)

                        cv2.imshow('res', imgori)
                        cv2.waitKey(1)

                pub_imageInfo.publish(imageExtract)

            # two lines situation
            elif len(linesExracted) == 2:
                a1, r1 = linesExracted[0]
                a2, r2 = linesExracted[1]

                imageExtract.rhs.append(r1)
                imageExtract.ths.append(a1)
                imageExtract.rhs.append(r2)
                imageExtract.ths.append(a2)

                # xcos() + ysin() = r
                ans = np.matrix([[np.cos(a1), np.sin(a1)], [np.cos(a2), np.sin(a2)]]).I
                # x0 y0 is the cross point of two lines
                x0 = int(ans[0, 0] * r1 + ans[0, 1] * r2)
                y0 = int(ans[1, 0] * r1 + ans[1, 1] * r2)

                imageExtract.nodePosX = x0
                imageExtract.nodePosY = y0

                # if the node is in the detect area 25% ~ 75%
                if abs(x0 - sizex / 2) <= sizex / 4 and abs(y0 - sizey / 2) <= sizey / 4:
                    rings = exractTopoStructor(img2v, x0, y0)

                    imageExtract.exitDirs = rings

                    if isShowingResult:
                        for th in rings:
                            rx = int(searchR * np.cos(th) + x0)
                            ry = int(searchR * np.sin(th) + y0)
                            cv2.line(imgori, (x0, y0), (rx, ry), (255, 0, 0), 3)
                            cv2.imshow('res', imgori)
                            cv2.waitKey(1)

                else:
                    if isShowingResult:
                        cv2.circle(imgori, (x0, y0), 10, (0, 0, 255), 5)
                        cv2.imshow('res', imgori)
                        cv2.waitKey(1)

                pub_imageInfo.publish(imageExtract)

            # strange situation
            else:
                print(linesExracted)

                for theta, rho in linesExracted:

                    a = np.cos(theta)
                    b = np.sin(theta)

                    x0 = a * rho
                    y0 = b * rho
                    x1 = int(x0 + 1000 * (-b))
                    y1 = int(y0 + 1000 * a)
                    x2 = int(x0 - 1000 * (-b))
                    y2 = int(y0 - 1000 * a)
                    cv2.line(imgori, (x1, y1), (x2, y2), (0, 255, 0), 2)

                if isShowingResult:
                    cv2.imshow('2vMap', img2v)
                    cv2.waitKey(1)
                    cv2.imshow('res', imgori)
                    cv2.waitKey(1)

        rate.sleep()


        # cv2.imwrite('after/' + pic, imgcut)

        # cv2.imshow('im', imgcut)
        # cv2.waitKey()
        #
        # cv2.imshow('im', imgori)
        # cv2.waitKey()
