# file: distance_width_yolov8.py
import cv2
from ultralytics import YOLO
import socket
import json

HOST = "192.168.1.138"   # IP del contenedor
PORT = 5005

def enviar_a_ros(clase, distancia, error_x):
    data = {
        "clase": clase,
        "distancia": float(distancia),
        "error_x": float(error_x)
    }

    msg = json.dumps(data)

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((HOST, PORT))
        sock.sendall(msg.encode())
        sock.close()
    except Exception as e:
        print("Error enviando al contenedor:", e)


def distance_to_camera(known_width, focal_length, per_width):
    if per_width <= 0:
        return None
    return (known_width * focal_length) / per_width


def focal_length_from_image(known_distance, known_width, bbox_width_px):
    return (bbox_width_px * known_distance) / known_width


# Ancho real por clase detectada (cm)
distancia_clases = {
    "Lampara": 21.0,
    "Mochila": 45.0
}

def sgn(x):
    return -1 if x < 0 else (1 if x > 0 else 0)


def main():
    print("Cargando modelo...")
    model = YOLO(r"/home/samuel/ros_melodic/src/pioneer_ws/src/best4.pt")

    # --- Calibración con imagen ---
    print("Cargando imagen de referencia...")
    ref_img = cv2.imread(r"/home/samuel/ros_melodic/src/pioneer_ws/src/ref.jpeg")

    if ref_img is None:
        print("ERROR: No se pudo leer ref.jpeg")
        return

    ref_result = model(ref_img)[0]

    if len(ref_result.boxes) == 0:
        print("ERROR: La imagen de referencia no detectó ningún objeto.")
        return

    # Tomar PRIMER bounding box detectado
    x1, y1, x2, y2 = ref_result.boxes.xyxy[0]
    ref_width_px = float(x2 - x1)

    # Parámetros reales
    KNOWN_DISTANCE = 70.0
    KNOWN_WIDTH = 31.0

    FOCAL_LENGTH = focal_length_from_image(KNOWN_DISTANCE, KNOWN_WIDTH, ref_width_px)

    print("Ancho bounding box ref:", ref_width_px)
    print("FOCAL =", FOCAL_LENGTH)

    # --- Video ---
    cap = cv2.VideoCapture(4)
    if not cap.isOpened():
        print("ERROR: No se pudo abrir la cámara.")
        return

    print("Cámara abierta. Presiona ESC para salir.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("ERROR: No se pudo leer frame.")
            break

        results = model(frame)[0]

        h, w, _ = frame.shape
        ch = (h - 1) / 2
        cw = (w - 1) / 2
        class_names  = []
        distances = []
        centro = []

        for box in results.boxes:
            cls_id = int(box.cls[0])
            class_name = results.names[cls_id]
            # Bounding box
            x1, y1, x2, y2 = box.xyxy[0]
            x1, y1, x2, y2 = float(x1), float(y1), float(x2), float(y2)

            width_px = x2 - x1
            cx = (x1 + x2) / 2
            cy = (y1 + y2) / 2

            print(f"Centro en eje x: {cx}")
            print(f"Centro en eje y: {cy}")
            print(f"Error en eje x: {cx - cw}")
            print(f"Error en eje y: {cy - ch}")

            known_width = distancia_clases[class_name]
            distance = (distance_to_camera(known_width, FOCAL_LENGTH, width_px) / 2) - 7
            if class_name not in distancia_clases:
                continue
            class_names.append(class_name)
            distances.append(distance)
            centro.append(cx)

            # Dibujar bounding box
            cv2.rectangle(frame,
                        (int(x1), int(y1)),
                        (int(x2), int(y2)),
                        (0, 255, 0), 2)

            cv2.putText(frame, class_name,
                        (int(x1), int(y1) - 25),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 255, 0), 2)

            if distance:
                cv2.putText(frame, f"{distance:.1f} cm",
                            (int(x1), int(y1) - 5),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (0, 255, 0), 2)

        cv2.imshow("YOLOv8 + Distancia (Ancho)", frame)
        if class_names:
            if "Mochila" in class_names and "Lampara" in class_names:
                indice_m = class_names.index("Mochila")
                indice_l = class_names.index("Lampara")
                signo = sgn (centro [indice_l] - centro [indice_m])
                enviar_a_ros(class_names [indice_l], distances [indice_l], centro [indice_l]-cw+ signo * cw * 0.5)
            else:
                enviar_a_ros(class_names [0], distances [0], centro [0]-cw)
        if cv2.waitKey(1) & 0xFF == 27:  # ESC
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

