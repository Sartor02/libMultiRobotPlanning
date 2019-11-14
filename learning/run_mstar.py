import exec_with_proc as rp

def run(map, timeout):
    cmd = "timeout {} mstar_public/cpp/main -i {} -o /tmp/mstar.result".format(timeout, map)
    retcode = rp.run_with_current_proc(cmd)
    if retcode != 0:
        print("M* timeout")
        return timeout
    f = open("/tmp/mstar.result")
    runtime = [float(x.strip().replace("runtime:", "")) for x in f.readlines() if "runtime:" in x][0]
    return runtime