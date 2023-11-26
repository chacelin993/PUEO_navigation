%                    MATLAB TUTORIAL for PHYS 660
%    Matrix operations, Programming, and Scientific Data Visualization
%--------------------------------------------------------------------------

% Keep the following rules in mind: (1) MATLAB is case sensitive; (2) typing name
% of a variable will cause MATLAB to display its current value; (3) a
% semicolon at the end of a command suppresses the screen output; (4) [],
% {}, and () are not interchangable; (5) the up and down arrow keys can be used 
% to scroll through your previous commands; (6) you can type help topic to access 
% online help on the command, function, or symbol topic; (7) if you press TAB key 
% after partially typing a function or a variable name, MATLAB will attempt to complete 
% it; (8) you can quit MATLAB by typing exit or quit.

ver
lookfor diag
help diag

% Basic floating-point (FP) arithmetics
% MATLAB uses double precision (64-bit) arithmetics
2+6
ans  % shows the most recent answer
4^2
1/0
pi*2
tan(pi/6)
exp(i*pi)
exp(j*pi)
va=2^(-24);
va
format long % tell MATLAB to display all 16 digits
va, sqrt(va)
realmax % largest positive real number
realmin % smallest positive real number
-2*realmax
0*Inf
eps % smallest number that can be added to 1 and produce number larger than 1
1+eps % eps quantifies the magnitude of roundoff errors
exp(log(10))-10 %in FP arithmetics the difference between "equal" values is nonzero 
format % go back to default format of 5 digits


% Basic vector operations
a=[1 2 3]
a(3)
a(1,3)
c=[4; 5; 6]
c(2)
c(2,1)
a*c
dot(a,c)
A=c*a
d=a
d(7)=0 %automatic storage allocation
a*a %this will generate error since the product of this type is not defined
transpose(a)
a*transpose(a)


%Elementwise operations on arrays using . combined with some operator
b=a.^2
a.*b
exp(a)
log(ans)
sqrt(a)
sum(b), mean(c)

%Colon operator
v=1:6
w=2:3:10, y=1:-0.25:0
B=[A;[1,2,3]]
C=[A,[8;9;10]]
D=[A;a]
C(2,3)
C(2:3,1:2)
G=D+B.*i
C(:,1)
C(1,:)

%Linspace and logspace
linspace(1,10,5) %generates 5 equally spaced points between 1 and 10
logspace(1,10,5) %logarithmically equally spaced points between decades 10^X1 and 10^X2

%List variables in the workspace
who
whos

clear


%Flow control
%-------------
%for loop example
g=1;
for k=1:7, g=g*k; end
g

%multiple for loops can be nested - use identation to improve readability
n=5;
for j=2:n
    for k=1:j-1
        A(k,j)=k/j;
        A(j,k)=k/j;
    end
end
A

%while loop
x=1; while x>0, xmin=x; x=x/2; end, xmin
%or with infinite while 1, ..., end (useful when it is not convenient to
%put the exit test at the top of the loop)
x=1;
while 1
    xmin=x;
    x=x/2;
    if x==0, break, end
end
xmin

%if statement (exmample employs "if" to swap two variables)
x=5, y=3
if x>y
    temp=y;
    y=x;
    x=temp;
end
x, y

%Plotting:
t=0:0.005:1; z=exp(10*t.*(t-1)).*sin(12*pi*t);
plot(t,z)
hist(rand(1000,1))


%toward using M-files: random Fibonacci sequence
clear
x=[1 2]
for n=2:999, x(n+1)=x(n)+sign(rand-0.5)*x(n-1); end
semilogy(1:1000,abs(x))
c=1.13198824;
hold on
semilogy(1:1000,c.^[1:1000])
hold off


%Linear algebra of matrices
B=[-3 0 -1; 2 5 -7; -1 4 8]
c=[4; 5; 6]
det(B)
rank(B)
rank([B c])
cond(B) % if condition number is not too large B matrix has an inverse with good numercial properties
help cond
x=inv(B)*c % texbook (slow)
norm(B*x-c)/(norm(B)-norm(x))
x=B\c %LU decomposition (fast)
norm(B*x-c)/(norm(B)-norm(x))
inv(B) %Matlab function to invert a matrix
inverse_B=eye(3)/B %LU decomposition and solving system of equations to compute inverse of a matrix

%Special matrices
I3=eye(3,3), Y=zeros(3,5), Z=ones(2)

%Random numbers
F=rand(3), G=randn(1,5), H=rand(1)
R = 1 + (7-1).*rand(10,1) %Generate values from the uniform distribution on the interval [1,7]

%Eigenproblem
sigma=[0 -i; i 0] %Pauli matrix for spin y
[V,D]=eig(sigma) %eigenvectors will be stored as columns of V-matrix and eigevalues along the diagonal of D-matrix
sigma.' %transpose
sigma'  %Hermitian conjugate = transpose + complex conjugate of all elements
norm(V'*V-eye(2)) %verify that matrix V of eigenvectors is a unitary matrix

%Spectral decomposition of diagonalizable matrix using Problem 1.6 from 
%L. E. Ballentine: "Quantum mechnanics: A modern development" as an example:
%Find the eigenvalus and eigenvectors of matrix M shown below, and  
%then construct the corresponding projection operators to verify the
%spectral decomposition theorem for this matrix.

M=[0 1 0; 1 0 1; 0 1 0] %matrix specified in the problem (it is Hermitian, as well as symmetric)
[MV,MD]=eig(M) %find eigenvalues and eigenvectors using MATLAB function eig
help eig
eigenval=diag(MD) %store eigenvalues into a vector
help diag %note that diag has several quite different functionalities 

projector1=MV(:,1)*MV(:,1)' %outer product of the first ket eigenvector and its bra gives projector on the corresponding eigensubspace
projector2=MV(:,2)*MV(:,2)' %outer product of the second ket eigenvector and its bra gives projector on the corresponding eigensubspace
projector3=MV(:,3)*MV(:,3)' %outer product of the third ket eigenvector and its bra gives projector on the corresponding eigensubspace

M_sd=eigenval(1)*projector1+eigenval(2)*projector2+eigenval(3)*projector3 %spectral decomposition of matrix M
M_trans=MV*MD*MV' %unitary transformation of from digonal representation to original representation 
M_exp=exp(eigenval(1))*MV(:,1)*MV(:,1)'+exp(eigenval(2))*MV(:,2)*MV(:,2)'+exp(eigenval(3))*MV(:,3)*MV(:,3)' %Exponent of matrix M using spectral decomposition
expm(M) %matrix exponential built-in into MATLAB
help expm
exp(M) %this has different meaning - it computes the exponential of a matrix element-by-element
M_exp_trans=MV*exp(MD)*MV' %attempt to get exponent of matrix M using unitary transformation - INCORRECT!
M_exp_trans=MV*diag(exp(eigenval))*MV' %this corrects mistake from the previous line
