import cv2


DEV_ID = 0

WIDTH = 640
HEIGHT = 480
FPS = 5


REC_SEC = 10

def main():
    cap = cv2.VideoCapture(DEV_ID)


    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, FPS)

    path = "./teat_camera.mp4"

    fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')
    out = cv2.VideoWriter(path, fourcc, FPS, (WIDTH, HEIGHT))


    for _ in range(FPS * REC_SEC):
        ret, frame = cap.read()
        out.write(frame)

    cap.release()
    out.release()
    cv2.destroyAllWindows()
    return


if __name__ == "__main__":
    main()