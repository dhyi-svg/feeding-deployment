from remote_molmo import RemoteMolmo


def main():
    molmo = RemoteMolmo(
        ssh_host="rj277@bhattacharjee-compute-02.coecis.cornell.edu",
        remote_dir="/home/rj277/molmo/sensor_msgs",
    )

    vis_image, pixel_coords, response = molmo.query(
        image_path="test.jpg",
        prompt="Point to start / 30 secs buttons",
        save_response_image_to="test_keypoint.jpg",
    )

    print("Pixel coords:", pixel_coords)
    print("Generated text:", response["generated_text"])


if __name__ == "__main__":
    main()