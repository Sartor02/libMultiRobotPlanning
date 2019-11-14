import exec_with_proc as rp
import glob

def get_line(ls, string, t):
    string = string.strip()
    if string[-1] != ':':
        string += ':'
    fls = [l for l in ls if string in l]
    if len(fls) > 1:
        print(string, "is not unique")
    elif len(fls) < 1:
        print(string, "is not found")
    assert(len(fls) == 1)
    line = fls[0]
    line = line.replace(string, '').replace(':', '').strip()
    return t(line)

def run(map, timeout):
    prefix = "/tmp/xstar_"
    cmd = "./collect_data_xstar.py {} {} {} {} {} .result"\
          .format(map,
                  timeout,
                  1,
                  30,
                  prefix)
    rp.run_with_current_proc(cmd)
    result_file = "/tmp/xstar_0.result"
    try:
        f = open(result_file, 'r')
        ls = f.readlines()
        bounds = get_line(ls, "successive_bounds", eval)
        times = get_line(ls, "successive_runtimes", eval)
        return (bounds, times)
    except:
        print("X* read failed!")
        return ([0], [timeout])