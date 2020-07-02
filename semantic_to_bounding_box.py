import cv2
import numpy as np 
import csv

#num_images = 'enter this'  (not required)
for imgs in glob.glob("../images/train/*.png"):
	image = cv2.imread(imgs,0)
	image_csv = imgs[:-3]+"csv"

	idx= np.where(image==255)
	image[idx]=0
	idx=np.where(image==6)
	image[idx]=255
	idx=np.where(image==7)
	image[idx]=255
	idx=np.where(image==11)
	image[idx]=255
	idx= np.where(image!=255)
	image[idx]=0
	#(image*255).astype(np.uint8)
	
	#contour finding
	#gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
	#ret, thresh = cv2.threshold(gray, 127, 255, 0)
	contours, hierarchy = cv2.findContours(gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	myData = [["CLASS","xmin","ymin", "xmax", "ymax"]]

	for contour in contours:
		x, y, w, h = cv2.boundingRect(contour)
		print(x,y,w,h)
		img = cv2.rectangle(image,(x,y),(x+w,y+h),(0,255,0),2)
		
		myData.append([img[(x+w)//2][(y+h)//2],x,y,x+w,y+h])
		with open(image_csv, 'w',newline='') as fFile:
			writer = csv.writer(fFile)
			writer.writerows(myData)

#cv2_imshow(image)
#cv2.waitKey(1000)