## LeRobot Record Interface

Simple Interface that uses Pygame to display images and texts. It also manages keyboard events.
This simple interface can only display two images of the same size side by side and a text in the middle bottom of the
screen.

## YAML Configuration

````YAML
nodes:
  - id: lerobot_record
    path: record.py # modify this to the relative path from the graph file to the client script
    inputs:
      tick: dora/timer/millis/16 # update the interface every 16ms (= 60fps)

      # image_left: some image from other node 
      # image_right: some image from other node
    outputs:
      - text
      - episode
      - failed
      - end # end signal that can be sent to other nodes (sent when the window is closed)

    env:
      WINDOW_WIDTH: 1280 # window width (default is 640) 
      WINDOW_HEIGHT: 1080 # window height (default is 480)
````

## Inputs

## Outputs:

- `text` : Array containing 1 element, the text in Arrow format.
- `episode` : Array containing 1 element, the episode number in Arrow format (or -1, marks episode end).
- `failed` : Array containing 1 element, the episode number failed in Arrow format.
- `end` : Array containing 1 empty element, indicates the end of recording to the dataflow.

## License

This library is licensed under the [Apache License 2.0](../../LICENSE).