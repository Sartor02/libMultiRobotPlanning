import exec_with_proc as rp

def run(map, timeout):
    cmd = "timeout {} release/cbs -i {} -o /tmp/cbs.result".format(timeout, map)
    retcode = rp.run_with_current_proc(cmd)
    if retcode != 0:
        print("CBS timeout")
        return timeout
    f = open("/tmp/cbs.result")
    runtime = [float(x.strip().replace("runtime:", "")) for x in f.readlines() if "runtime:" in x][0]
    return runtime