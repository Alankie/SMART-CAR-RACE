#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import torch, os
from Timer import *
from _02PipeDatasetLoader import *
from _03Unet import *
# from _21CalEvaluationIndicator import *
from PIL import Image
from _33Warpimg import *

time_init = timer(8)
time_loadnet = timer(6)

Device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
# Device='cpu'
print(Device)

# print(Device)
# %% 载入数据、模型
# FolderPath = '..\Dataset'
# TrainDataset, TrainDataLoader, ValDataset, ValDataLoader = PipeDatasetLoader(FolderPath, 1)
# Unet_BCELoss_Adam
ModelFolder ='/home/scut1/catkin_ws/src/csi_cam_test/scripts/line/Output'
# ModelFolder = './Output'
# SaveFolder = './Dataset/Val'

# Unet = UNet(in_channels=3, out_channels=1, init_features=4, WithActivateLast=True, ActivateFunLast=torch.sigmoid).to(
#    Device)
Unet = UNet(in_channels=3, out_channels=1, init_features=4, WithActivateLast=True,
            ActivateFunLast=torch.sigmoid).to(Device)

# time_loadnet('loadnet')
# convert_net=timer(9)
# Unet=Unet.to(Device)
# convert_net('convert_the_net')

Unet.load_state_dict(torch.load(os.path.join(ModelFolder, 'checkpoint7.pt'), map_location=Device))  # 700
Unet.eval()
# 测试模式
torch.set_grad_enabled(False)

time_init('finish time_init_NET')

InputImgSize = (128, 128)
ValImgTransform = transforms.Compose([
    transforms.Resize(InputImgSize),
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.46], std=[0.10]),
])


def from_camera_to_net(img):
    img = warpimg(img)
    # cv2.imwrite('warp.jpg',img)
    # to_tensor=timer(5)
    img3 = Image.fromarray(img)
    FusionImg = img3
    FusionImg = ValImgTransform(FusionImg)
    # print('31fusion', FusionImg.shape)
    image_to_net = torch.unsqueeze(FusionImg, 0)
    image_to_net = image_to_net.to(Device)
    # print('image.net tpye', type(image_to_net))
    # to_tensor('img_to_tensor')
    # time_in_net = timer(5)
    OutputImg2 = Unet(image_to_net)
    # time_in_net('time in net0')
    # time_astype=timer(6)
    OutputImg2 = OutputImg2.cpu().detach().numpy()[0, 0]
    OutputImg2 = (OutputImg2 * 255).astype(np.uint8)
    # time_astype('time as type')
    # cv2.imshow('output', OutputImg2)
    # cv2.waitKey(0)
    # cv2.imwrite(('a.png'),OutputImg2)
    # print('return out2',type(OutputImg2))
    return OutputImg2


if __name__ == '__main__':
    all_t = timer(8)
    try_img = cv2.imread('forwarp.jpg')
    cv2.imshow('oriimg', try_img)
    # try_img2=warpimg(try_img)
    # cv2.imwrite('warp.png',try_img2)
    time_in_net = timer(5)
    out = from_camera_to_net(try_img)
    time_in_net('time in net00')
    cv2.imwrite('out.png', out)
    cv2.imshow('out', (160, 120))
    cv2.waitKey(1)
    print('net success in find the line')
    # chage_from_camera(FolderPath,50)
    all_t('end')
