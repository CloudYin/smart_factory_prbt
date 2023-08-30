#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from smart_factory.srv import GetBoxPenCenter, GetBoxPenCenterResponse


class ImageProcessing:
    def __init__(self):
        rospy.init_node("image_processing")
        self.bridge_ = CvBridge()
        self.cv_image_ = None
        self.image_subscriber_ = rospy.Subscriber(
            "pylon_camera_node/image_rect_color", Image, self.image_callback
        )
        self.image_processing_server_ = rospy.Service(
            "get_box_pen_center", GetBoxPenCenter, self.callback_GetBoxPenCenter
        )

    def image_callback(self, data):
        try:
            self.cv_image_ = self.bridge_.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge error: %s", e)

    def callback_GetBoxPenCenter(self, req):
        box_centers_y = []
        pen_centers_y = []
        box_plate_lower_roi = self.cv_image_[350:740, 1170:1540]
        box_plate_upper_roi = self.cv_image_[70:350, 1170:1540]
        box_plate_roi = self.cv_image_[70:740, 1170:1540]
        pen_plate_roi = self.cv_image_[70:720, 70:620]

        box_upper_roi_gray = cv2.cvtColor(box_plate_upper_roi, cv2.COLOR_BGR2GRAY)
        box_lower_roi_gray = cv2.cvtColor(box_plate_lower_roi, cv2.COLOR_BGR2GRAY)
        pen_roi_gray = cv2.cvtColor(pen_plate_roi, cv2.COLOR_BGR2GRAY)

        _, box_upper_mask = cv2.threshold(
            box_upper_roi_gray, 128, 255, cv2.THRESH_BINARY
        )
        _, box_lower_mask = cv2.threshold(
            box_lower_roi_gray, 90, 255, cv2.THRESH_BINARY
        )
        _, pen_mask = cv2.threshold(pen_roi_gray, 128, 255, cv2.THRESH_BINARY)

        box_upper_contours, box_upper_hierarchy = cv2.findContours(
            box_upper_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE
        )

        box_lower_contours, box_lower_hierarchy = cv2.findContours(
            box_lower_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE
        )

        pen_contours, pen_hierarchy = cv2.findContours(
            pen_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE
        )

        # Select inner contours
        try:
            box_upper_hierarchy = box_upper_hierarchy[0]
            for component in zip(box_upper_contours, box_upper_hierarchy):
                currentContour = component[0]
                currentHierarchy = component[1]
                if currentHierarchy[3] == -1:
                    # these are the outermost child components
                    img_contour = cv2.drawContours(
                        box_plate_upper_roi, currentContour, -1, (0, 0, 0)
                    )
                    if 50000 < cv2.moments(currentContour)["m00"] < 100000:
                        m00 = cv2.moments(currentContour)["m00"]
                        m01 = cv2.moments(currentContour)["m01"]
                        m10 = cv2.moments(currentContour)["m10"]
                        centerX = round(m10 / m00)
                        centerY = round(m01 / m00)
                        x, y, w, h = cv2.boundingRect(currentContour)
                        points = np.array(
                            [[[x, y]], [[x + w, y]], [[x + w, y + h]], [[x, y + h]]]
                        )
                        cv2.drawContours(img_contour, [points], -1, (0, 255, 0), 2)
                        box_center_y = (15 - centerY * 0.2687) / 1000
                        box_centers_y.append(box_center_y)
                        cv2.circle(
                            img_contour, (int(centerX), int(centerY)), 5, (0, 0, 255)
                        )
            cv2.imwrite(
                "/home/pilz/Pictures/smart_factory/box_contour_upper.jpg", img_contour
            )
        except:
            pass

        try:
            box_lower_hierarchy = box_lower_hierarchy[0]
            for component in zip(box_lower_contours, box_lower_hierarchy):
                currentContour = component[0]
                currentHierarchy = component[1]
                if currentHierarchy[3] == -1:
                    # these are the outermost child components
                    img_contour = cv2.drawContours(
                        box_plate_lower_roi, currentContour, -1, (0, 0, 0)
                    )
                    if 50000 < cv2.moments(currentContour)["m00"] < 100000:
                        m00 = cv2.moments(currentContour)["m00"]
                        m01 = cv2.moments(currentContour)["m01"]
                        m10 = cv2.moments(currentContour)["m10"]
                        centerX = round(m10 / m00)
                        centerY = round(m01 / m00)
                        x, y, w, h = cv2.boundingRect(currentContour)
                        points = np.array(
                            [[[x, y]], [[x + w, y]], [[x + w, y + h]], [[x, y + h]]]
                        )
                        cv2.drawContours(img_contour, [points], -1, (0, 255, 0), 2)
                        box_center_y = (17 - centerY * 0.2687 - 73) / 1000
                        box_centers_y.append(box_center_y)
                        cv2.circle(
                            img_contour, (int(centerX), int(centerY)), 5, (0, 0, 255)
                        )
            cv2.imwrite(
                "/home/pilz/Pictures/smart_factory/box_contour_lower.jpg", img_contour
            )
        except:
            pass
        box_centers_y = sorted(box_centers_y)

        pen_hierarchy = pen_hierarchy[0]
        for component in zip(pen_contours, pen_hierarchy):
            currentContour = component[0]
            currentHierarchy = component[1]
            if currentHierarchy[3] == -1:
                # these are the outermost child components
                img_contour = cv2.drawContours(
                    pen_plate_roi, currentContour, -1, (0, 0, 0)
                )
                if 16000 < cv2.moments(currentContour)["m00"] < 18000:
                    m00 = cv2.moments(currentContour)["m00"]
                    m01 = cv2.moments(currentContour)["m01"]
                    m10 = cv2.moments(currentContour)["m10"]
                    centerX = round(m10 / m00)
                    centerY = round(m01 / m00)
                    x, y, w, h = cv2.boundingRect(currentContour)
                    points = np.array(
                        [[[x, y]], [[x + w, y]], [[x + w, y + h]], [[x, y + h]]]
                    )
                    cv2.drawContours(img_contour, [points], -1, (0, 255, 0), 2)
                    pen_center_y = (14 - centerY * 0.2687) / 1000
                    pen_centers_y.append(pen_center_y)
                    cv2.circle(
                        img_contour, (int(centerX), int(centerY)), 5, (0, 0, 255)
                    )
            cv2.imwrite(
                "/home/pilz/Pictures/smart_factory/pen_contour.jpg", img_contour
            )
        # cv2.imshow("img_contour", pen_image)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        pen_centers_y = sorted(pen_centers_y)

        return GetBoxPenCenterResponse(box_centers_y, pen_centers_y)


if __name__ == "__main__":
    ImageProcessing()
    rospy.spin()
