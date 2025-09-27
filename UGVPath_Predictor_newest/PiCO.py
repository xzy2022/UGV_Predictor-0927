import torch
import torch.nn as nn
import numpy as np
import torch.nn.functional as F
from global_define import *


Queue_Length = 1024
moco_m = 0.999
proto_m = 0.99


class PiCO(nn.Module):
    def __init__(self, base_encoder):
        super().__init__()
        self.encoder_q = base_encoder()
        self.encoder_q.initialize_weights()
        self.encoder_k = base_encoder()
        for param_q, param_k in zip(self.encoder_q.parameters(), self.encoder_k.parameters()):
            # 将param_q的参数复制给param_k，确保两个网络初始化参数相同
            param_k.data.copy_(param_q.data)
            param_k.requires_grad = False  # k网络不进行反向传播，因此其参数不计算梯度

        # create the queue
        # quene, 大小为8192*64
        self.register_buffer("queue", torch.randn(Queue_Length, Feature_Dim))
        # queue_GT_Y, 大小为8192*1 （标签）
        self.register_buffer("queue_GT_Y", torch.randn(Queue_Length, 1))
        # queue_ptr：大小为1，类型为long
        self.queue = F.normalize(self.queue, dim=0)
        self.register_buffer("queue_ptr", torch.zeros(1, dtype=torch.long))
        # prototypes： 大小为2*64
        self.register_buffer("prototypes", torch.zeros(Class_Num, Feature_Dim))

    # update momentum encoder，对k网络参数进行更新的方法（该方法不计算梯度，也不进行反向传播）
    @torch.no_grad()
    def _momentum_update_key_encoder(self):
        # 利用zip()函数将q网络和k网络对应位置的参数打包成元组列表[(weight_11_q, weight_11_k), ... ,(bias_11_q, bias_11_k), ...]
        # 后遍历这个列表，并将每一对参数值赋值给param_q和param_k，之后利用q网络的参数对k网络的参数进行更新
        for param_q, param_k in zip(self.encoder_q.parameters(), self.encoder_k.parameters()):
            # 更新公式为：\theta_k = m \theta_k + (1-m) \theta_q
            param_k.data = param_k.data * moco_m + param_q.data * (1. - moco_m)

    @torch.no_grad()
    # 函数输入：特征features，预测得分labels，标签partial_Y
    def _dequeue_and_enqueue(self, features, labels):
        batch_size = features.shape[0]
        ptr = int(self.queue_ptr)
        new_ptr = ptr + batch_size
        # assert Queue_Length % batch_size == 0
        if new_ptr < Queue_Length:
            # 输入的features尺寸是B*64，quene的尺寸为8192*64，也就是从当前位置ptr开始，将features填充到队列中ptr到ptr+B-1这B个位置
            self.queue[ptr:ptr + batch_size, :] = features
            self.queue_GT_Y[ptr:ptr + batch_size, :] = labels
            # 移动队列的当前位置（向前移动B个位置）
            ptr = (ptr + batch_size) % Queue_Length  # move pointer
            self.queue_ptr[0] = ptr
        else:
            ptr_first = Queue_Length - ptr
            ptr_second = new_ptr - Queue_Length
            self.queue[ptr:ptr + ptr_first, :] = features[0:ptr_first]
            self.queue_GT_Y[ptr:ptr + ptr_first, :] = labels[0:ptr_first]
            # 移动队列的当前位置（向前移动ptr_first个位置）
            ptr = (ptr + ptr_first) % Queue_Length  # move pointer
            self.queue[ptr:ptr + ptr_second, :] = features[ptr_first:batch_size]
            self.queue_GT_Y[ptr:ptr + ptr_second, :] = labels[ptr_first:batch_size]
            # 移动队列的当前位置（向前移动ptr_second个位置）
            ptr = (ptr + ptr_second) % Queue_Length  # move pointer
            self.queue_ptr[0] = ptr

    def forward(self, feature_map_batch, mask_road_batch, GT_Y, b_test=False):
        feature_env, feature_q, output = self.encoder_q(feature_map_batch, mask_road_batch)

        if b_test is False:
            # 利用输入的特征进行原型更新
            with torch.no_grad():
                for feat, label in zip(feature_q, GT_Y):
                    # 对第label个原型进行更新，同样是动量的更新方式，即p_l = p_l*m + (1-m)*feat
                    self.prototypes[label] = self.prototypes[label] * proto_m + (1 - proto_m) * feat
            # 对更新后的原型进行二范数归一化
            self.prototypes = F.normalize(self.prototypes, p=2, dim=1)
            with torch.no_grad():
                self._momentum_update_key_encoder()
            _, feature_k, _ = self.encoder_k(feature_map_batch, mask_road_batch)
            big_features = torch.cat((feature_q, feature_k, self.queue.clone().detach()), dim=0)
            GT_Y = GT_Y.unsqueeze(1)
            big_GT_Y = torch.cat((GT_Y, GT_Y, self.queue_GT_Y.clone().detach()), dim=0)
            self._dequeue_and_enqueue(feature_k, GT_Y)
            return feature_env, big_features, big_GT_Y, output
        else:
            return feature_env, output, feature_q
