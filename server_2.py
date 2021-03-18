import numpy as np
import cv2
from flask import Flask
from flask_restful import Resource, Api, reqparse
import json
import numpy as np
import base64
from model_script import detection_ml
# compression
import zlib
import time
import codecs
from PIL import Image
dml = detection_ml()
app = Flask(__name__)
api = Api(app)
parser = reqparse.RequestParser()
parser.add_argument('imgb64', help = 'type error')


class Predict(Resource):

    def ret_prediction(self,f):
        faces,locs = dml.detect_face(f)
        print(faces)
        return dml.detect_mask(faces)

    def post(self):
        data = parser.parse_args()
        #print(data)
        if data['imgb64'] == "":

            return {
                    'data':'',
                    'message':'No file found',
                    'status':'error'
                    }




        #img = open(data['imgb64'], 'r').read() # doesn't work
        img = data['imgb64']


        data2 = img.encode()
        data2 = base64.b64decode(data2)
        data2 = zlib.decompress(data2)
        fdata = np.frombuffer(data2, dtype=np.uint8)

        if fdata.size == 0:
            return {
                    'data':'',
                    'message':'empty array found',
                    'status':'error'
                    }


        fdata = fdata.reshape(-1,300,400,3)
        v = time.time()
        print('fdata_shape',fdata.shape)
        image = fdata[0]
        img = Image.fromarray(image, 'RGB')
        img.show()
        pred_list = [self.ret_prediction(f) for f in fdata]
        print('length preds',len(pred_list[0]),len(pred_list[1]))
        v2 = time.time()
        time_lag = v2-v       
        pred_list = np.array(pred_list)
        print(pred_list)
        mask_tot = pred_list[:,:,0].sum()
        not_mask_tot = pred_list[:,:,1].sum()

        val = 'not mask'

        if mask_tot > not_mask_tot:
            val = 'mask' 

        print(mask_tot,not_mask_tot)     

        if img:
            return json.dumps(
                {
                'data':time_lag,
                'message':val,
                'status':'worked'
                }

)
        return {
                'data':'',
                'message':'Something when wrong',
                'status':'error'
                }


api.add_resource(Predict,'/predict')

if __name__ == '__main__':
    app.run(debug=True, host = '0.0.0.0', port = 5000, threaded=True)