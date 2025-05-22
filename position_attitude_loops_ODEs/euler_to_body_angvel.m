function Q = euler_to_body_angvel(alpha, beta)

Q = [ 1  0          -sin(beta);
     0  cos(alpha)  cos(beta)*sin(alpha);
     0  -sin(alpha)  cos(beta)*cos(alpha)];

end
