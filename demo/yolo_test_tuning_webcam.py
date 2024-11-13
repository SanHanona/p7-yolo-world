import cv2
import supervision as sv
import time 


from tqdm import tqdm
from inference.models.yolo_world.yolo_world import YOLOWorld

import string
import nltk
from nltk import word_tokenize, pos_tag

import ssl

try:
    _create_unverified_https_context = ssl._create_unverified_context
except AttributeError:
    pass
else:
    ssl._create_default_https_context = _create_unverified_https_context

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


model = YOLOWorld(model_id="yolo_world/s") #can be l, m, s (large, medium, small)
# classes = ['hand']

classes = extract_noun_phrases('man with blue')
model.set_classes(classes)

cap = cv2.VideoCapture(0)
avg_confidence=0
avg_fps=0

t1 = 0

if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

frame_count = 0
while True:
    ret, frame = cap.read()
    if not ret:
        break

    results = model.infer(frame, confidence=0.05)
    detections = sv.Detections.from_inference(results).with_nms(threshold=0.1)

    labels = [
        f"{classes[class_id]} {confidence:0.3f}"
        for class_id, confidence
        in zip(detections.class_id, detections.confidence)
    ]

    annotated_image = frame.copy()
    annotated_image = BOUNDING_BOX_ANNOTATOR.annotate(annotated_image, detections)
    annotated_image = LABEL_ANNOTATOR.annotate(annotated_image, detections, labels=labels)


    #calculate the frame-rate    
    t2 = time.time()
    fps = 1/(t2-t1)
    t1 = t2

    avg_fps=(avg_fps+fps)/2

    try:
        avg_confidence = (avg_confidence + detections.confidence[0])/2
    except:
        pass


    

    #convert the frame-rate into an integer and then to a string, so it can be displayed on the image
    fps=int(fps)
    fps=str(fps)

    cv2.putText(annotated_image, fps, (7, 70), cv2.FONT_HERSHEY_SIMPLEX, 2, (100, 255, 0), 3, cv2.LINE_AA)

    # Display the resulting frame
    cv2.imshow('Annotated Video', annotated_image)

    # Press 'q' to exit the video stream
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything is done, release the capture
#print(detections.confidence)
print("average confidence", avg_confidence)
print("average framerate", avg_fps)
cap.release()
cv2.destroyAllWindows()
