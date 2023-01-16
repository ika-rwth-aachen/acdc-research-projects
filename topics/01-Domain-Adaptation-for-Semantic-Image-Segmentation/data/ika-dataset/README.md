## ika Dataset for Semantic Image Segmentation

### :arrow_forward: [Download](https://rwth-aachen.sciebo.de/s/xkPs4OPBdtuA6YF)

### Folder Structure

Original camera images are stored in `input`, corresponding semantic segmentation label images are stored in `label`.

```
ika-dataset
├── rgb_to_class_id.py
├── input
│   ├── 2_frame_000159.png
│   ├── 2_frame_001798.png
│   └── ...
└── label
    ├── 2_frame_000159_gtFine_labelColor.png
    ├── 2_frame_001798_gtFine_labelColor.png
    └── ...
```

### Color-Coding

The color-coding of semantic classes is a reduced version of the one used in the [Cityscapes dataset](https://www.cityscapes-dataset.com/). Some semantic classes of the original dataset are reduced to a common class, e.g., the classes *rider*, *bicycle*, and *motorcycle* are joined under the common class *two wheeler*. This class reduction is illustrated below.

![](color-coding-reduction.png)

In `rgb_to_class_id.py` you can find a Python dictionary defining this class reduction. Note that the semantic segmentation labels of this dataset are already reduced, i.e., only the classes/colors on the right side of the illustration are present.
