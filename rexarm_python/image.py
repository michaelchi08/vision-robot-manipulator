import cv2
import numpy as np
import freenect

if __name__ == "__main__":

	currentVideoFrame = np.array([])
	currentDepth10Frame = np.array([])

	currentVideoFrame = freenect.sync_get_video()[0]
	currentDepth10Frame = freenect.sync_get_depth()[0]

	h, w, _ = currentVideoFrame.shape

	cv2.namedWindow('Color', cv2.WINDOW_AUTOSIZE)
	cv2.namedWindow('Depth10', cv2.WINDOW_AUTOSIZE)
	cv2.namedWindow('Depth8', cv2.WINDOW_AUTOSIZE)

	counter = 0
	
	while True:

		
		currentVideoFrame = cv2.cvtColor(freenect.sync_get_video()[0],cv2.COLOR_RGB2BGR)
		currentDepth10Frame = freenect.sync_get_depth()[0]
		currentDepth8Frame = currentDepth10Frame

		currentDepth8Frame >>= 2
		currentDepth8Frame = currentDepth8Frame.astype(np.uint8)
		
		cv2.imshow("Window", currentVideoFrame) 					
		cv2.imshow("Depth10", currentDepth10Frame)
		cv2.imshow("Depth8", currentDepth8Frame)

		ch = cv2.waitKey(10)

		# press 'esc' to exit
		if ch == 0x1B:
			break
		#Press 'u' to save an image 
		if ch == 0x75:
			print("Saving images: %d" % counter)
			cv2.imwrite("color-%d.png"%counter, currentVideoFrame)
			cv2.imwrite("depth_10-%d.png"%counter, currentDepth10Frame)
			cv2.imwrite("depth_8-%d.png"%counter, currentDepth8Frame)
			counter+=1
