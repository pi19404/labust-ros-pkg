#!/usr/bin/env python
import subprocess as sub
import threading

class RunCmd(threading.Thread):
    def __init__(self, cmd, timeout):
        threading.Thread.__init__(self)
        self.cmd = cmd
        self.timeout = timeout

    def run(self):
        self.p = sub.Popen(self.cmd)
        self.p.wait()

    def Run(self):
        self.start()
        self.join(self.timeout)

        if self.is_alive():
            self.p.terminate()
            self.join()

if __name__ == "__main__":
    step = 0.1;
    inner_start = 0.1;
    outer_start = 0.1;
    inner_final = 1.0;
    outer_final = 1.0;
   
    print "T:",int(outer_final/step);
    print "Range:",range(2)
    
    
    for i in range(int(outer_final/step)+1):
        for j in range(i,int(inner_final/step)+1):
            outer = outer_start + step*i;
            inner = inner_start + step*j;
            sub.call(["./do_batch.sh",str(inner),str(outer)]);
            sub.call(["rospack","find","caddy_uwsim"]);
            RunCmd(["roslaunch", "simulation_batch.launch"], 240).Run();
    
    