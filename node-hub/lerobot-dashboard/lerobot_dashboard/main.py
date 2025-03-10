"""
This Dora node is a minimalistic interface that shows two images and text in a Pygame window.
"""

import os
import argparse

import numpy as np
import pygame

import pyarrow as pa

from dora import Node


def main():
    # Handle dynamic nodes, ask for the name of the node in the dataflow
    parser = argparse.ArgumentParser(
        description="LeRobot Record: This node is used to record episodes of a robot interacting with the environment."
    )

    parser.add_argument(
        "--name",
        type=str,
        required=False,
        help="The name of the node in the dataflow.",
        default="lerobot_record",
    )
    parser.add_argument(
        "--window-width",
        type=int,
        required=False,
        help="The width of the window.",
        default=640,
    )
    parser.add_argument(
        "--window-height",
        type=int,
        required=False,
        help="The height of the window.",
        default=480,
    )

    args = parser.parse_args()

    window_width = int(os.getenv("WINDOW_WIDTH", args.window_width))
    window_height = int(os.getenv("WINDOW_HEIGHT", args.window_height))

    image_left = pygame.Surface((int(window_width // 2), window_height // 2))
    image_right = pygame.Surface((int(window_width // 2), window_height // 2))

    pygame.font.init()
    font = pygame.font.SysFont("Comic Sans MS", 30)
    text = font.render("No text to render", True, (255, 255, 255))

    pygame.init()

    screen = pygame.display.set_mode((window_width, window_height + text.get_height()))

    pygame.display.set_caption("Pygame minimalistic interface")

    node = Node(args.name)

    episode_index = 1
    recording = False

    for event in node:
        event_type = event["type"]
        if event_type == "STOP":
            break

        elif event_type == "INPUT":
            event_id = event["id"]

            if event_id == "image_left":
                arrow_image = event["value"][0]

                image = {
                    "width": arrow_image["width"].as_py(),
                    "height": arrow_image["height"].as_py(),
                    "channels": arrow_image["channels"].as_py(),
                    "data": arrow_image["data"].values.to_numpy(),
                }

                image_left = pygame.image.frombuffer(
                    image["data"], (image["width"], image["height"]), "BGR"
                )

            elif event_id == "image_right":
                arrow_image = event["value"][0]
                image = {
                    "width": arrow_image["width"].as_py(),
                    "height": arrow_image["height"].as_py(),
                    "channels": arrow_image["channels"].as_py(),
                    "data": arrow_image["data"].values.to_numpy(),
                }

                image_right = pygame.image.frombuffer(
                    image["data"], (image["width"], image["height"]), "BGR"
                )

            elif event_id == "tick":
                node.send_output("tick", pa.array([]), event["metadata"])

                running = True
                for pygame_event in pygame.event.get():
                    if pygame_event.type == pygame.QUIT:
                        running = False
                        break
                    elif pygame_event.type == pygame.KEYDOWN:
                        key = pygame.key.name(pygame_event.key)

                        if key == "space":
                            recording = not recording
                            if recording:
                                text = font.render(
                                    f"Recording episode {episode_index}",
                                    True,
                                    (255, 255, 255),
                                )

                                node.send_output(
                                    "episode",
                                    pa.array([episode_index]),
                                    event["metadata"],
                                )
                            else:
                                text = font.render(
                                    f"Stopped recording episode {episode_index}",
                                    True,
                                    (255, 255, 255),
                                )

                                node.send_output(
                                    "episode",
                                    pa.array([-1]),
                                    event["metadata"],
                                )

                                episode_index += 1

                        elif key == "return":
                            if recording:
                                recording = not recording
                                text = font.render(
                                    f"Failed episode {episode_index}",
                                    True,
                                    (255, 255, 255),
                                )

                                node.send_output(
                                    "failed",
                                    pa.array([episode_index]),
                                    event["metadata"],
                                )
                                episode_index += 1
                                node.send_output(
                                    "episode",
                                    pa.array([-1]),
                                    event["metadata"],
                                )

                            elif episode_index >= 2:
                                text = font.render(
                                    f"Failed episode {episode_index - 1}",
                                    True,
                                    (255, 255, 255),
                                )

                                node.send_output(
                                    "failed",
                                    pa.array([episode_index - 1]),
                                    event["metadata"],
                                )

                if not running:
                    break

                screen.fill((0, 0, 0))

                # Draw the left image
                screen.blit(image_left, (0, 0))

                # Draw the right image
                screen.blit(image_right, (window_width // 2, 0))

                # Draw the text bottom center
                screen.blit(
                    text,
                    (window_width // 2 - text.get_width() // 2, int(window_height)),
                )

                pygame.display.flip()

        elif event_type == "ERROR":
            raise ValueError("An error occurred in the dataflow: " + event["error"])

    node.send_output("end", pa.array([]))

    pygame.quit()


if __name__ == "__main__":
    main()
