import os, cv2, random
import numpy as np

width, height = 640, 480

shapes = ['circle', 'square', 'triangle']
colors = {
    'white':(255,255,255),
    'black': (0, 0, 0),
    'red': (0, 0, 255),
    'green': (0, 255, 0),
    'blue': (255, 0, 0),
    'yellow': (0, 255, 255),
}

classes = [f"{color}_{shape}" for shape in shapes for color in colors]

def draw_shape(image, label):
    shape = label.split('_')[1]
    color_name = label.split('_')[0]
    color = colors[color_name]

    x = random.randint(100, 540)
    y = random.randint(100, 380)
    size = random.randint(40, 80)

    if shape == 'circle':
        cv2.circle(image, (x, y), size, color, -1)
    elif shape == 'square':
        cv2.rectangle(image, (x-size, y-size), (x+size, y+size), color, -1)
    elif shape == 'triangle':
        pts = np.array([[x, y-size], [x-size, y+size], [x+size, y+size]], np.int32)
        cv2.drawContours(image, [pts], 0, color, -1)

    return x, y, size*2, size*2

def create_dataset(path, count_per_class=50):
    os.makedirs(f'{path}/images/train', exist_ok=True)
    os.makedirs(f'{path}/labels/train', exist_ok=True)

    for label in classes:
        for i in range(count_per_class):
            img = np.full((height, width, 3), 100, np.uint8) 
            x, y, w, h = draw_shape(img, label)

            img_name = f'{label}_{i}.jpg'
            txt_name = f'{label}_{i}.txt'
            img_path = f'{path}/images/train/{img_name}'
            label_path = f'{path}/labels/train/{txt_name}'
            cv2.imwrite(img_path, img)

            xc, yc = x / width, y / height
            w, h = w / width, h / height
            class_id = classes.index(label)
            with open(label_path, 'w') as f:
                f.write(f"{class_id} {xc:.6f} {yc:.6f} {w:.6f} {h:.6f}\n")

    # create data.yaml
    with open(f'{path}/data.yaml', 'w') as f:
        f.write(f"path: {path}\n")
        f.write("train: images/train\n")
        f.write("val: images/train\n")
        f.write("names:\n")
        for i, name in enumerate(classes):
            f.write(f"  {i}: {name}\n")

if __name__ == '__main__':
    create_dataset('dataset_colors')
