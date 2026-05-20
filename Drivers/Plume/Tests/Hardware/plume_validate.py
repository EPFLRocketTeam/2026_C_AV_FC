
import argparse
import struct

def hello_world (args):
    return b"Hello, World !\n" + bytes([0])
def data_buffer (args):
    buffer = []

    for i in range(256 * 256):
        j = ((i ^ 0b1101100110111000) << 16) | i

        buffer.append(struct.pack("<I", j))
    
    return b"".join(buffer)

def expected_buffer (args):
    total_buffer          = hello_world(args) + data_buffer(args)
    post_truncated_length = len(total_buffer) - len(total_buffer) % (args.block_size - args.header_size)

    return total_buffer[:post_truncated_length]

def main ():
    parser = argparse.ArgumentParser(
        "Plume Manual Test - Validator",
        description="Take the output of 'plume_cli ... ls X' and verify its content"
    )
    parser.add_argument("file")
    parser.add_argument("--header-size", default=8, type=int)
    parser.add_argument("--block-size", default=512, type=int)

    args = parser.parse_args()

    with open(args.file, "rb") as file:
        content = file.read()
    
    expected = expected_buffer(args)

    if len(expected) != len(content):
        print("ERROR: Wrong length")
        print(" - expected:", len(expected))
        print(" - found:", len(content))

        min_length = min(len(expected), len(content))
        print()
        print("Doing tests with length", min_length)
    
        expected = expected[:min_length]
        content  = content[:min_length]

    if expected != content:
        print("ERROR: they are different")
        print()
        for idx in range(len(expected)):
            if expected[idx] != content[idx]:
                print("First different index", idx)
                print(" - Expected: ", expected[idx:idx + 64])
                print(" - Content: ", content[idx:idx + 64])
                break
    else:
        print("OK")

if __name__ == "__main__":
    main()
