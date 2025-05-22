function [X] = Schur_solver(A, B, Q, R)

% Construct the Hamiltonian matrix
H = [A, -B*(R\B'); 
    -Q, -A'];

% Compute the Schur decomposition
[U, S] = schur(H, 'real');  % Real Schur decomposition
select = real(diag(S)) < 0;  % Select only stable eigenvalues
[U, ~] = ordschur(U, S, select);

% Separate the stable subspace
n = size(A,1);
U11 = U(1:n,1:n);
U21 = U(n+1:end,1:n);

% Compute the solution X = U21 * U11^(-1)
X = U21 / U11;

