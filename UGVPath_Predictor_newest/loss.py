
import torch
import torch.nn as nn
from global_define import *


class SupConLoss(nn.Module):
    def __init__(self, temperature=0.07, base_temperature=0.07):
        super().__init__()
        self.temperature = temperature
        self.base_temperature = base_temperature

    def forward(self, features, mask, batch_size):
        # SupCon loss (Partial Label Mode)
        mask = mask.float().detach().cuda()
        # compute logits
        anchor_dot_contrast = torch.div(torch.matmul(features[:batch_size], features.T), self.temperature)
        # for numerical stability
        logits_max, _ = torch.max(anchor_dot_contrast, dim=1, keepdim=True)
        logits = anchor_dot_contrast - logits_max.detach()
        # mask-out self-contrast cases
        logits_mask = torch.scatter(torch.ones_like(mask), 1, torch.arange(batch_size).view(-1, 1).cuda(), 0)
        mask = mask * logits_mask
        # compute log_prob
        exp_logits = torch.exp(logits) * logits_mask
        log_prob = logits - torch.log(exp_logits.sum(1, keepdim=True) + 1e-12)
        sum_mask = mask.sum(1)
        log_likelihood = (mask * log_prob).sum(1)
        sum_mask_filtered = sum_mask[np.where((sum_mask.cpu()) != 0)]
        log_likelihood_filtered = log_likelihood[np.where((sum_mask.cpu()) != 0)]
        # compute mean of log-likelihood over positive
        mean_log_prob_pos = log_likelihood_filtered / sum_mask_filtered
        # mean_log_prob_pos_old = (mask * log_prob).sum(1) / mask.sum(1)
        # loss_old = - (self.temperature / self.base_temperature) * mean_log_prob_pos_old
        # loss
        loss = - (self.temperature / self.base_temperature) * mean_log_prob_pos
        loss = loss.mean()
        # print(loss)
        return loss


class BCELoss_class_weighted(nn.Module):
    def __init__(self, weight):
        super().__init__()
        self.weight = weight #二分类中正负样本的权重，第一项为负类权重，第二项为正类权重

    def forward(self, input, target):
        input = torch.clamp(input, min=1e-7, max=1-1e-7)
        bce = - self.weight[1] * target * torch.log(input) - (1 - target) * self.weight[0] * torch.log(1 - input)
        return torch.mean(bce)


# class Frechet_Loss(nn.Module):
#     def __init__(self):
#         super().__init__()
#
#     def forward(self, input, target):
#         input_copy = input.cpu().detach().numpy()
#         target_copy = target.cpu().detach().numpy()
#         B = input_copy.shape[0]
#         similarity_sum = 0
#         for i in range(B):
#             target_i = target_copy[i, :]
#             input_i = input_copy[i, :]
#             # target_c_i = target_copy[i, :, 0]
#             # target_r_i = target_copy[i, :, 1]
#             # target_x_i = (target_c_i - cols_ / 2) * resolution
#             # target_y_i = (rows_ / 2 - 1 - target_r_i) * resolution
#             # target_i[:, 0] = target_x_i
#             # target_i[:, 1] = target_y_i
#             similarity_i = shape_similarity(input_i, target_i)
#             similarity_sum = similarity_sum + similarity_i
#         similarity_average = similarity_sum/B
#         loss_frechet = 1 - similarity_average
#         return loss_frechet
#         # print(loss_frechet)