# Copyright 2019 Google LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""A demo that runs object detection on camera frames using OpenCV.

TEST_DATA=../all_models

Run face detection model:
python3 detect.py \
  --model ${TEST_DATA}/mobilenet_ssd_v2_face_quant_postprocess_edgetpu.tflite

Run coco model:
python3 detect.py \
  --model ${TEST_DATA}/mobilenet_ssd_v2_coco_quant_postprocess_edgetpu.tflite \
  --labels ${TEST_DATA}/coco_labels.txt

"""
from flask import Response
from flask import Flask
from flask import render_template
import threading
import argparse
import datetime
import imutils
import time

import argparse
import cv2
import os

from pycoral.adapters.common import input_size
from pycoral.adapters.detect import get_objects
from pycoral.utils.dataset import read_label_file
from pycoral.utils.edgetpu import make_interpreter
from pycoral.utils.edgetpu import run_inference


outputFrame = None
lock = threading.Lock()

# initialize a flask object
app = Flask(__name__)


@app.route("/")
def index():
	# return the rendered template
	return render_template("index.html")

def capture_v(args):
	global outputFrame, lock

	print('Loading {} with {} labels.'.format(args.model, args.labels))
	interpreter = make_interpreter(args.model)
	interpreter.allocate_tensors()
	labels = read_label_file(args.labels)
	inference_size = input_size(interpreter)

	cap = cv2.VideoCapture(args.camera_idx)
	# cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
	# Sony PS3 EYE cam settings:
	# 320x240 @ 125 FPS, 640x480 @ 60 FPS, 320x240 @187 FPS --> use excat FSP setting
	cap.set(cv2.CAP_PROP_FPS, 60)
	cap.set(cv2.CAP_PROP_FRAME_WIDTH,320),
	cap.set(cv2.CAP_PROP_FRAME_HEIGHT,240)
	size = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
		int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
	print("image size=",size)
	
	fps=0
	start_time = time.time()

	while cap.isOpened():
		ret, frame = cap.read()
		if not ret:
			break
		cv2_im = frame

		cv2_im_rgb = cv2.cvtColor(cv2_im, cv2.COLOR_BGR2RGB)
		cv2_im_rgb = cv2.resize(cv2_im_rgb, inference_size)
		run_inference(interpreter, cv2_im_rgb.tobytes())
		objs = get_objects(interpreter, args.threshold)[:args.top_k]
		cv2_im = append_objs_to_img(cv2_im, inference_size, objs, labels)
		with lock:
			outputFrame = cv2_im
		fps+=1
		if fps==200:
			end_time = time.time()
			print("cam FPS:",fps/(end_time-start_time))
			start_time = time.time()
			fps=0


	cap.release()



def generate():
	# grab global references to the output frame and lock variables
	global outputFrame, lock

	fps=0
	start_time = time.time()
	# loop over frames from the output stream
	while True:
		fps+=1
		if fps==200:
			end_time = time.time()
			print("web FPS:",fps/(end_time-start_time))
			start_time = time.time()
			fps=0
		# wait until the lock is acquired
		with lock:
			# check if the output frame is available, otherwise skip
			# the iteration of the loop
			if outputFrame is None:
				continue
			# encode the frame in JPEG format
			(flag, encodedImage) = cv2.imencode(".jpg", outputFrame)
			# ensure the frame was successfully encoded
			if not flag:
				continue

		# yield the output frame in the byte format
		yield(b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + 
			bytearray(encodedImage) + b'\r\n')
		time.sleep(0.1) # slow down web FPS

@app.route("/video_feed")
def video_feed():
	# return the response generated along with the specific media
	# type (mime type)
	return Response(generate(),
		mimetype = "multipart/x-mixed-replace; boundary=frame")

def main():
	default_model_dir = '/home/pi/dev/coral/examples/examples-camera/all_models'
	# default_model = 'mobilenet_ssd_v2_coco_quant_postprocess_edgetpu.tflite'
	default_model = 'mobilenet_ssd_v1_coco_quant_postprocess_edgetpu.tflite' #works faster for person detection
	default_labels = 'coco_labels.txt'
	parser = argparse.ArgumentParser()
	parser.add_argument('--model', help='.tflite model path',
						default=os.path.join(default_model_dir,default_model))
	parser.add_argument('--labels', help='label file path',
						default=os.path.join(default_model_dir, default_labels))
	parser.add_argument('--top_k', type=int, default=3,
						help='number of categories with highest score to display')
	parser.add_argument('--camera_idx', type=int, help='Index of which video source to use. ', default = 0)
	parser.add_argument('--threshold', type=float, default=0.6,
						help='classifier score threshold')
	args = parser.parse_args()

	# start a thread that will perform motion detection
	t = threading.Thread(target=capture_v, args=(args,))
	t.daemon = True
	t.start()

	# start the flask app
	app.run(host="0.0.0.0", port=8000, debug=True,
		threaded=True, use_reloader=False)


def append_objs_to_img(cv2_im, inference_size, objs, labels):
    height, width, channels = cv2_im.shape
    scale_x, scale_y = width / inference_size[0], height / inference_size[1]
    for obj in objs:
        bbox = obj.bbox.scale(scale_x, scale_y)
        x0, y0 = int(bbox.xmin), int(bbox.ymin)
        x1, y1 = int(bbox.xmax), int(bbox.ymax)

        percent = int(100 * obj.score)
        label = '{}% {}'.format(percent, labels.get(obj.id, obj.id))

        cv2_im = cv2.rectangle(cv2_im, (x0, y0), (x1, y1), (0, 255, 0), 2)
        cv2_im = cv2.putText(cv2_im, label, (x0, y0+30),
                             cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)
    return cv2_im

if __name__ == '__main__':
    main()
