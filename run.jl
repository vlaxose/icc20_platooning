include("createRandomSystem.jl")
include("primalADMM.jl")
A,B=createRandomSystem("a1")
B11 = B[1:5,1]
B13 = B[1:5,3]
yi = randn(6,1)
Aeq = zeros(5, 6, 2)
Aeq[:,:,1] = [A[1:5,1:5] B11]
Aeq[:,:,2] = [zeros(5,5) B13]
Beq_i = Aeq[:,:,1]*yi
R = zeros(1,1,3)
R[:,:,1] = Matrix(1I, 1, 1);

primalADMM(Aeq, Beq_i, Q, R, Matrix(1I, 5, 5), [1,3], [1,2], 1, 3)
