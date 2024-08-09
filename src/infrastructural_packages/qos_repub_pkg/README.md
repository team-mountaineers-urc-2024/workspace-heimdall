# QOS Repub Package

## Description
This Package contains two nodes. The first (compressed_repub) subscribes to a CompressedImage topic and republishes the same information, but as best effort instead of reliable. The second (decompress_repub) subscribes to a CompressedImage topic and republishes the same information, but as reliable instead of best_effort. The purpose of this node was to allow for better data transmission over lossy networks when using nodes that were not created by us.

## Known Limitations
This Package can only republish CompressedImage topics, there was a plan to rewrite it to convert any message type from reliable to best effor and vice versa, but this did not pan out.