import cv2
import supervision as sv

from tqdm import tqdm
from inference.models.yolo_world.yolo_world import YOLOWorld

import string
import nltk
from nltk import word_tokenize, pos_tag
# import ssl

# try:
#     _create_unverified_https_context = ssl._create_unverified_context
# except AttributeError:
#     pass
# else:
#     ssl._create_default_https_context = _create_unverified_https_context

nltk.download()

nltk.download('punkt')
nltk.download('averaged_perceptron_tagger')


def extract_noun_phrases(text):
    
    tokens = word_tokenize(text)
    tokens = [token for token in tokens if token not in string.punctuation]
    tagged = pos_tag(tokens)
    print(tagged)
    grammar = 'NP: {<DT>?<JJ.*>*<NN.*>+}'
    cp = nltk.RegexpParser(grammar)
    result = cp.parse(tagged)
    
    noun_phrases = []
    for subtree in result.subtrees():
        if subtree.label() == 'NP':
            noun_phrases.append(' '.join(t[0] for t in subtree.leaves()))
    
    return noun_phrases

BOUNDING_BOX_ANNOTATOR = sv.BoundingBoxAnnotator(thickness=2)
LABEL_ANNOTATOR = sv.LabelAnnotator(text_thickness=2, text_scale=1, text_color=sv.Color.BLACK)

SOURCE_VIDEO_PATH = f"data/slow_traffic_small.mp4"
video_capture = cv2.VideoCapture(SOURCE_VIDEO_PATH)

model = YOLOWorld(model_id="yolo_world/s")

# classes = ["chair","table","shelf","person"]

classes = extract_noun_phrases('white car')
model.set_classes(classes)

if not video_capture.isOpened():
    print("Error: Could not open video.")
else:
    while True:
        ret, frame = video_capture.read()
        if not ret:
            break

        results = model.infer(frame, confidence=0.2)
        detections = sv.Detections.from_inference(results).with_nms(threshold=0.85)

        annotated_image = frame.copy()
        annotated_image = BOUNDING_BOX_ANNOTATOR.annotate(annotated_image, detections)
        annotated_image = LABEL_ANNOTATOR.annotate(annotated_image, detections)

        # Display the resulting frame
        cv2.imshow('Annotated Video', annotated_image)

        # Press 'q' to exit the video stream
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything is done, release the capture
    video_capture.release()
    cv2.destroyAllWindows()



