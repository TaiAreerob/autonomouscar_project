import glob
import csv
import os
import sys
import cv2

input_path = 'E:/TL/images'
output_file_name = 'E:/TL/tl_labeling_seoul.csv'
f = open(output_file_name, 'wt', encoding='utf-8')
f.write("count,folder_name,picture_name,tl_type,x,y,width,height\n")    

cnt = 1
dir_names = os.listdir(input_path)
for dir_name in dir_names:
    full_path = os.path.join(input_path, dir_name, 'anno', 'bbox')

    for input_file in glob.glob(os.path.join(full_path,'*.csv')):
        with open(input_file, 'r', newline='') as csv_in_file:
            filereader = csv.reader(csv_in_file)
            for row in filereader:
                file_name = os.path.basename(input_file)[:-4]
                path_name = os.path.basename(file_name)
                line = str(cnt)+","+str(dir_name)+","+str(file_name)+","+str(row[0])+","+str(row[1])+","+str(row[2])+","+str(row[3])+","+str(row[4])+'\n'
                f.write(line)
                cnt += 1

f.close()