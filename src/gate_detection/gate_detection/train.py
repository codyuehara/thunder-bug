import os
import json
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
from torchvision import transforms
from PIL import Image
import numpy as np

from unet import UNet   # import the UNet you already have

class GateDataset(Dataset):
    def __init__(self, images_dir, ann_file, transform=None, size=356):
        self.images_dir = images_dir
        self.ann = json.load(open(ann_file))
        self.transform = transform
        self.size = size
    
        self.image_id_to_file = {im["id"]: im["file_name"] for im in self.ann["images"]}
        self.masks = self._build_masks()

    def _build_masks(self):
        """Convert polygon annotations into binary masks per image."""
        pass

    def train():
        pass
