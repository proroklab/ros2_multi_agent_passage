import cv2
import argparse
import numpy as np
from pathlib import Path


def process_video(video_in_path, video_out_path):
    cap_in = cv2.VideoCapture(video_in_path)
    cap_in.set(cv2.CAP_PROP_POS_FRAMES, 240)

    video_out = cv2.VideoWriter(
        video_out_path,
        cv2.VideoWriter_fourcc(*"mp4v"),
        50,  # FPS
        (1920, 1080),
    )

    last_frame = None
    aggr = None
    frame_i = 0

    overlay = np.zeros((1080, 1920, 4), dtype=np.uint8)

    while cap_in.isOpened():
        # Capture frame-by-frame
        ret, frame = cap_in.read()
        frame_i += 1
        # if frame_i == 10:
        #    break
        if ret == True:
            print(frame_i)

            frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            if last_frame is not None:
                #diff = np.abs(frame_gray.astype(np.int) - last_frame)
                #diff[diff < 20] = 0
                #if aggr is None:
                #    aggr = diff.astype(np.uint8)
                #else:
                #    aggr = np.clip(aggr + diff, 0, 255).astype(np.uint8)

                # threshold for bright spots
                fr_br = (frame_gray > 240).astype(float)

                blur = cv2.GaussianBlur(frame_gray, (5, 5), 0)
                ret3, th3 = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

                cv2.imshow("Frame", th3)

                kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
                # closing: remove "negative" noise (close pixels under threshold)
                closing = cv2.morphologyEx(fr_br, cv2.MORPH_CLOSE, kernel)
                # opening: remove "positive" noise (true noise that is not actually the LED)
                opening = cv2.morphologyEx(closing, cv2.MORPH_OPEN, kernel)
                # dilation: open up to include area around the LED
                kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
                dilation = cv2.dilate(opening, kernel, iterations=4)

                # overlay[dilation > 0] = frame[dilation > 0]
                # fr_br = cv2.dilate(fr_br, kernel, iterations=1)

                # smoothen out the resulting blob
                kernel = np.ones((9, 9), np.float32) / 9 ** 2
                dst = cv2.filter2D(dilation, -1, kernel)
                dst = cv2.filter2D(dst, -1, kernel)

                dst_bgra = np.repeat(dst[..., np.newaxis], 4, axis=2)
                frame_alpha = cv2.cvtColor(frame, cv2.COLOR_BGR2BGRA)

                overlay = np.maximum(overlay, (dst_bgra * frame_alpha).astype(np.uint8))
                frame_ov = (
                    frame
                    * np.repeat(1.0 - (overlay[..., 3, np.newaxis] / 255), 3, axis=2)
                ).astype(np.uint8)
                frame_ov += overlay[..., :3]
                frame_final = frame_ov #cv2.flip(frame_ov, 1)
                # cv2.imshow("Frame", frame_final)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

                video_out.write(frame_final)
            last_frame = frame_gray

        # Break the loop
        else:
            break

    video_out.release()
    cap_in.release()
    cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("video_in_path")
    parser.add_argument("video_out_path")
    args = parser.parse_args()

    process_video(args.video_in_path, args.video_out_path)


if __name__ == "__main__":
    main()
