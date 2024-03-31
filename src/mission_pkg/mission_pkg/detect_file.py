#!/usr/bin/env python3
import cv2
import ApriltagModule

def READtoGRY(path):
    img_path=path
    tag_img = cv2.imread(img_path)
    tag_gry = cv2.cvtColor(tag_img,cv2.COLOR_BGR2GRAY)
    return tag_gry

def main():
    tag_gray = READtoGRY('/home/yusuf/april/gazebo_apriltag/models/Apriltag36_11_00001/materials/textures/tag36_11_00001.png')
    #tag_gray = READtoGRY('/home/yusuf/tag1.png')
    #print(type(tag_gray))
    detector = ApriltagModule.Detector()
    detections = detector.detect(tag_gray)

    for tag in detections:
            print("Tag ID:", tag.tag_id)
            print("Tag Koordinatları:", tag.center)
            print("Tag Köşeleri:", tag.corners)
            # Etiket boyutlarını belirlemek için köşe noktaları arasındaki uzaklığı hesapla
            tag_width = max(tag.corners[:, 0]) - min(tag.corners[:, 0])
            tag_height = max(tag.corners[:, 1]) - min(tag.corners[:, 1])
            print("Tag Genişliği:", tag_width)
            print("Tag Yüksekliği:", tag_height)
            print("Tag Açıları:", tag.homography)



if __name__ == '__main__':
    main()



