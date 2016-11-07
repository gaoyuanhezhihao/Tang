import argparse
parser = argparse.ArgumentParser()
parser.add_argument("log_file", type=str,
                    help="display a square of a given number")
args = parser.parse_args()

with open(args.log_file, 'r') as f:
    lines = f.readlines()
    f.close()
key_tokens = ['l_ok', 'r_ok', 'D_ok', 'B_ok']
last_token = ''
for line in lines:
    for tk in key_tokens:
        if tk is not last_token and tk in line:
            print line
