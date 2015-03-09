classdef VarClass
%Took some code from http://people.math.sc.edu/howard/Classes/726/inmat.html
    properties
        val
        var
    end
    
    methods
        
        function obj = VarClass(val,var)
            if nargin ~= 0
                validateattributes(val,{'numeric'},{'2d'});
                validateattributes(var,{'numeric'},{'size',size(val),'nonnegative'});

                m = size(val,1);
                n = size(val,2);
                obj(m,n) = VarClass;
                for i = 1:m
                    for j = 1:n
                        obj(i,j).val = double(val(i,j));
                        obj(i,j).var = double(var(i,j));
                    end
                end
            end
        end

        function out = plus(A,B)

            if(isnumeric(A))
                A = VarClass(A,zeros(size(A)));
            else
                validateattributes(A,{'VarClass'},{'2d'});
            end
            if(isnumeric(B))
                B = VarClass(B,zeros(size(B)));
            else
                validateattributes(B,{'VarClass'},{'size',size(A)});
            end
            
            valOut = zeros(size(A));
            valOut(:) = [A.val] + [B.val];
            varOut = zeros(size(A));
            varOut(:) = [A.var] + [B.var];
            out = VarClass(valOut,varOut);
        end

        function out = minus(A,B)

            if(isnumeric(A))
                A = VarClass(A,zeros(size(A)));
            else
                validateattributes(A,{'VarClass'},{'2d'});
            end
            if(isnumeric(B))
                B = VarClass(B,zeros(size(B)));
            else
                validateattributes(B,{'VarClass'},{'size',size(A)});
            end
            
            valOut = zeros(size(A));
            valOut(:) = [A.val] - [B.val];
            varOut = zeros(size(A));
            varOut(:) = [A.var] + [B.var];
            out = VarClass(valOut,varOut);
        end
        
        function out = uminus(A)
            
            if(isnumeric(A))
                A = VarClass(A,zeros(size(A)));
            else
                validateattributes(A,{'VarClass'},{'2d'});
            end
            
            valOut = zeros(size(A));
            valOut(:) = -[A.val];
            varOut = zeros(size(A));
            varOut(:) = [A.var];
            out = VarClass(valOut,varOut);
        end

        function out = times(A,B)

            if(isnumeric(A))
                A = VarClass(A,zeros(size(A)));
            else
                validateattributes(A,{'VarClass'},{'2d'});
            end
            if(isnumeric(B))
                B = VarClass(B,zeros(size(A)));
            else
                validateattributes(B,{'VarClass'},{'size',size(A)});
            end
            
            valOut = zeros(size(A));
            valOut(:) = [A.val] .* [B.val];
            varOut = zeros(size(A));
            varOut(:) = [A.var] .* [B.var] + ([A.val].^2).*[B.var] + ([B.val].^2).*[A.var];
            out = VarClass(valOut,varOut);
        end      

        function out = mtimes(A,B)

            if(isnumeric(A))
                A = VarClass(A,zeros(size(A)));
            else
                validateattributes(A,{'VarClass'},{'2d'});
            end
            if(isnumeric(B))
                B = VarClass(B,zeros(size(A)));
            else
                validateattributes(B,{'VarClass'},{'nrows',size(A,2)});
            end
            
            out = repmat(VarClass(0,0),size(A,1),size(B,2));

            for i = 1:size(A,1)
                for j = 1:size(B,2)
                    for k = 1:size(A,2)
                        out(i,j).val = A(i,k).val .* B(k,j).val;
                        out(i,j).var = A(i,k).var .* B(k,j).var + (A(i,k).val.^2).*B(k,j).var + (B(k,j).val.^2).*A(i,k).var;
                    end
                end
            end
        end    

        function out = power(A,p)

            if(isnumeric(A))
                A = VarClass(A,zeros(size(A)));
            else
                validateattributes(A,{'VarClass'},{'2d'});
            end

            out = A;
            for i = 2:p
                out = out .* A;
            end
     
        end     

        function out = mpower(A,p)

            if(isnumeric(A))
                A = VarClass(A,zeros(size(A)));
            else
                validateattributes(A,{'VarClass'},{'scalar'});
            end
            validateattributes(p,{'numeric'},{'scalar','positive','integer'})

            out = A;
            for i = 2:p
                out = out .* A;
            end
        end   

        function out = rdivide(A,B)

            if(isnumeric(A))
                A = VarClass(A,zeros(size(A)));
            else
                validateattributes(A,{'VarClass'},{'2d'});
            end
            if(isnumeric(B))
                B = VarClass(B,zeros(size(B)));
            else
                validateattributes(B,{'VarClass'},{'size',size(A)});
            end
            
            varOut = zeros(size(A));
            varOut(:) = [B.var]./([B.val].^4);
            valOut = zeros(size(A));
            valOut(:) = 1./[B.val];
            
            varOut(:) = [A.var] .* varOut(:)' + ([A.val].^2).*varOut(:)' + (valOut(:)'.^2).*[A.var];
            valOut(:) = [A.val] .* valOut(:)';
            
            out = VarClass(valOut,varOut);
        end

        function out = mrdivide(A,B)

            if(isnumeric(A))
                A = VarClass(A,zeros(size(A)));
            else
                validateattributes(A,{'VarClass'},{'scalar'});
            end
            if(isnumeric(B))
                B = VarClass(B,zeros(size(B)));
            else
                validateattributes(B,{'VarClass'},{'scalar'});
            end

            tempVal = 1./B.val;
            tempVar = B.var./(B.val.^4);

            valOut = A.val .* tempVal;
            varOut = A.var .* tempVar + (A.val.^2).*tempVar + (tempVal.^2).*A.var;
            out = VarClass(valOut,varOut);
        end
        
        function out = inv(A)
            %only going to do the 3x3 case
            validateattributes(A,{'VarClass'},{'size',[3,3]});
            
            d = A(1,1).*A(2,2).*A(3,3) - A(1,1).*A(3,2).*A(2,3) - A(2,1).*A(1,2).*A(3,3) + A(2,1).*A(3,2).*A(1,3) + A(3,1).*A(1,2).*A(2,3) - A(3,1).*A(2,2).*A(1,3);
            out = [A(2,2)*A(3,3) - A(3,2)*A(2,3), A(3,2)*A(1,3) - A(1,2)*A(3,3), A(1,2)*A(2,3) - A(2,2)*A(1,3); A(3,1)*A(2,3) - A(2,1)*A(3,3), A(1,1)*A(3,3) - A(3,1)*A(1,3), A(2,1)*A(1,3) - A(1,1)*A(2,3); A(2,1)*A(3,2) - A(3,1)*A(2,2), A(3,1)*A(1,2) - A(1,1)*A(3,2), A(1,1)*A(2,2) - A(2,1)*A(1,2)];
            out = out./repmat(d,3,3);
        end
        
        function out = mldivide(A,B)

            if(isnumeric(A))
                A = VarClass(A,zeros(size(A)));
            else
                validateattributes(A,{'VarClass'},{'2d'});
            end
            if(isnumeric(B))
                B = VarClass(B,zeros(size(B)));
            else
                validateattributes(B,{'VarClass'},{'2d'});
            end

            temp = inv(A)*B;

            valOut = reshape([A.val],size(A)) \ reshape([B.val],size(B));
            varOut = reshape([temp.var],size(temp));
            out = VarClass(valOut,varOut);
        end
    end
end