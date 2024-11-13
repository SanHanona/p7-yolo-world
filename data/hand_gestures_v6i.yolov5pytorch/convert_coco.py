from pathlib import Path

import globox


def main() -> None:
  path = Path("train/labels/")  # Where the .txt files are
  save_file = Path("coco.json")

  annotations = globox.AnnotationSet.from_yolo_darknet(path, image_folder="train/images/")
  annotations.save_coco(save_file, auto_ids=True)


  ##### convert validation folder 
  # path = Path("valid/labels/")  # Where the .txt files are
  # save_file = Path("coco.json")

  # annotations = globox.AnnotationSet.from_yolo_darknet(path, image_folder="valid/images/") #
  # annotations.save_coco(save_file, auto_ids=True)




if __name__ == "__main__":
    main()