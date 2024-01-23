import torch
import torchvision
import transformers
import pytorch_lightning
import glob
from common import get_id2label_from_annotations_file


class CocoDetection(torchvision.datasets.CocoDetection):
    def __init__(self, img_folder, ann_file, feature_extractor):
        super(CocoDetection, self).__init__(img_folder, ann_file)
        self.feature_extractor = feature_extractor

    def __getitem__(self, idx):
        # read in PIL image and target in COCO format
        img, target = super(CocoDetection, self).__getitem__(idx)

        # preprocess image and target (converting target to DETR format, resizing + normalization of both image and target)
        image_id = self.ids[idx]
        target = {'image_id': image_id, 'annotations': target}
        encoding = self.feature_extractor(images=img, annotations=target, return_tensors="pt")
        pixel_values = encoding["pixel_values"].squeeze() # remove batch dimension
        target = encoding["labels"][0] # remove batch dimension

        return pixel_values, target


class YoloS(pytorch_lightning.LightningModule):
    def __init__(self, model, train_dataloader, val_dataloader, lr, weight_decay, batch_size):
        super().__init__()
        # replace COCO classification head with custom head
        self.model = model
        # see https://github.com/PyTorchLightning/pytorch-lightning/pull/1896
        self.lr = lr
        self.weight_decay = weight_decay
        self.batch_size = batch_size
        self.train_dataloaderr = train_dataloader
        self.val_dataloaderr = val_dataloader

    def forward(self, pixel_values):
        outputs = self.model(pixel_values=pixel_values)

        return outputs

    def common_step(self, batch, batch_idx):
        pixel_values = batch["pixel_values"]
        labels = [{k: v.to(self.device) for k, v in t.items()} for t in batch["labels"]]

        outputs = self.model(pixel_values=pixel_values, labels=labels)

        loss = outputs.loss
        loss_dict = outputs.loss_dict

        return loss, loss_dict

    def training_step(self, batch, batch_idx):
        loss, loss_dict = self.common_step(batch, batch_idx)
        # logs metrics for each training_step
        self.log("loss", loss, prog_bar=True, on_epoch=True, batch_size=self.batch_size)
        return loss

    def validation_step(self, batch, batch_idx):
        loss, loss_dict = self.common_step(batch, batch_idx)
        self.log("val_loss", loss, prog_bar=True, batch_size=self.batch_size)
        if batch_idx == 0:
            print()
        return loss

    def configure_optimizers(self):
        return torch.optim.AdamW(
            self.parameters(),
            lr=self.lr,
            weight_decay=self.weight_decay,
        )

    def train_dataloader(self):
        return self.train_dataloaderr

    def val_dataloader(self):
        return self.val_dataloaderr


class ModelCheckpointCallback(pytorch_lightning.Callback):
    def __init__(self, start_from_epoch):
        super().__init__()
        self.start_from_epoch = start_from_epoch

    def on_train_epoch_end(self, trainer, pl_module):
        folder = f"./checkpoints/epoch_{trainer.current_epoch + self.start_from_epoch}"
        pl_module.model.save_pretrained(folder)


HUGGINGFACE_PATH = "hustvl/yolos-small"
BATCH_SIZE = 1

def main():
    # load datasets
    feature_extractor = transformers.YolosImageProcessor.from_pretrained(
        HUGGINGFACE_PATH,
        size={"width": 1024, "height": 768},
    )
    root = 'assigns'
    train_dataset = CocoDetection(img_folder=root, ann_file='assigns/train_annotations.coco.json', feature_extractor=feature_extractor)
    val_dataset = CocoDetection(img_folder=root, ann_file='assigns/test_annotations.coco.json', feature_extractor=feature_extractor)
    print("Number of training examples:", len(train_dataset))
    print("Number of validation examples:", len(val_dataset))

    # create dataloaders
    def collate_fn(batch):
        pixel_values = [item[0] for item in batch]
        encoding = feature_extractor.pad(pixel_values, return_tensors="pt")
        return {
            "pixel_values": encoding['pixel_values'],
            "labels": [item[1] for item in batch],
        }
    train_dataloader = torch.utils.data.DataLoader(train_dataset, collate_fn=collate_fn, batch_size=BATCH_SIZE, num_workers=15, shuffle=True)
    val_dataloader = torch.utils.data.DataLoader(val_dataset, collate_fn=collate_fn, batch_size=BATCH_SIZE, num_workers=15)

    # setup id2label
    id2label = get_id2label_from_annotations_file()
    print("TRAINING WITH", len(id2label), "LABELS")

    # model setup
    epochs = glob.glob("./checkpoints/epoch_*")
    if len(epochs) == 0:
        start_from_epoch = 0
        model_inner = transformers.YolosForObjectDetection.from_pretrained(
            HUGGINGFACE_PATH,
            num_labels=len(id2label),
            ignore_mismatched_sizes=True,
        )
    else:
        last_epoch = sorted(epochs, key=lambda e: int(e.split("_")[-1]))[-1]
        start_from_epoch = int(last_epoch.split("_")[-1]) + 1
        model_inner = transformers.YolosForObjectDetection.from_pretrained(
            last_epoch,
            num_labels=len(id2label),
            ignore_mismatched_sizes=True,
        )
    print("STARTING FROM EPOCH", start_from_epoch)
    model = YoloS(model_inner, train_dataloader, val_dataloader, lr=0.8e-5, weight_decay=2e-4, batch_size=BATCH_SIZE)

    # training setup
    torch.set_float32_matmul_precision('medium')
    checkpoint_callback = ModelCheckpointCallback(start_from_epoch)
    trainer = pytorch_lightning.Trainer(
        max_epochs=50,
        gradient_clip_val=0.1,
        accumulate_grad_batches=12,
        log_every_n_steps=5,
        callbacks=[checkpoint_callback],
        enable_model_summary=False,
    )

    # train!
    try:
        trainer.fit(model)
    except ValueError as e:
        print(e)
        exit(36)

if __name__ == "__main__":
    main()
