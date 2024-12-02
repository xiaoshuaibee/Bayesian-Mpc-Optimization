import matlab
import matlab.engine
eng = matlab.engine.start_matlab()
#t = eng.mtest(4,2)
m = eng.untitled(1,1,1,1)
print(m)
#print(t)
