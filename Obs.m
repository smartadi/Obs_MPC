%% calc obs matrix
function Obsp = Obs(sys,eps,N,C,x_nom,u_nom,y_nom)

n = size(sys.A,1);
l = size(sys.C,1);
m = size(sys.A,1);
eps = 0.1;
x0 = x_nom(:,1);
x_init = x0+eps*[1*eye(n),-1*eye(n)];

x_obs=[];
y_obs=[];
for j  = 1:length(x_init)
    xo = zeros(n,N);
    yo = zeros(l,N);
    xo(:,1) = x_init(:,j);
    for i = 1:N
        xo(:,i+1) = sys.A*xo(:,i) + sys.B*u_nom(:,i);
        yo(:,i) = C(xo(:,i));
    end
    x_obs = [x_obs;xo];
    y_obs = [y_obs;yo];
     
end


Obsp = zeros(4,4);
for j = 1:N
    Y=[]; 
    for i = 1:n
          Y = [Y,(y_obs((i-1)*2+1:(i-1)*2+l,j) - y_obs(l*n+(i-1)*2+1:l*n+(i-1)*2+l,j))];
          
%           (y_obs((i-1)*2+1:(i-1)*2+l,j) - y_obs(l*n+(i-1)*2+1:l*n+(i-1)*2+l,j))
    end
    Obsp = Obsp + Y'*Y;
end
