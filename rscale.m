function Nbar = rscale(A, B, K)
    C = [1 0];  
    sys = ss(A - B*K, B, C, 0, 0.1);  
    dcgain_val = dcgain(sys);
    Nbar = 1 / dcgain_val;
end