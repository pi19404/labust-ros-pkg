function imgf = box_car(magnitude)
[n,m]=size(magnitude);

guard=5;
guardB = guard;
smpl=30;
smplB=10;
cellS=30;
cellB=10;


data = magnitude;
imgf=data;

magnitude(find(magnitude == -Inf)) = 0;

leftIdx = 1;
rightIdx =1;
leftIdxB = 1;
rightIdxB = 1;


for i=1:cellB:m
    rightB=i+cellB;
    if (rightB>m)
        rightB=m;
    end
    leftnB = [];
    rightnB= [];
    
   if (i>guardB)
       leftIdxB = i - guardB - smplB;
       if (leftIdxB<1) 
           leftIdxB = 1; 
       end;
    end
        
    if (i<(m-cellB-guardB))
       rightIdxB = i + cellB + guardB + smplB;
       if (rightIdxB>n) 
          rightIdxB = n; 
       end;
    end
        
    for j=1:cellS:n
        leftn = [];
        rightn = [];
        
        rightR=j+cellS;
        if (rightR>n)
            rightR=n;
        end
        
        leftnB = data(j:rightR,leftIdxB:leftIdxB+smplB);
        rightnB = data(j:rightR,(rightIdxB-smplB):rightIdxB);
        
        if (j>guard)
            leftIdx = j - guard - smpl;
            if (leftIdx<1) 
                leftIdx = 1; 
            end;
            leftn = data(leftIdx:leftIdx+smpl,leftIdxB:rightIdxB);
        end
        
        if (j<(n-cellS-guard))
            rightIdx = j + cellS + guard + smpl;
            if (rightIdx>n) 
                rightIdx = n; 
            end;
            rightn = data((rightIdx-smpl):rightIdx,leftIdxB:rightIdxB);
        end
               
        estn=[leftn;rightn];
        mn = mean2(estn);
        sd = std2(estn);
        %sd=1;
        %sm=sum(sum(estn));
        
        if (sd==0) sd=1;end;
        
        rightIdx = j+cellS;
        if (rightIdx>n) 
          rightIdx = n; 
        end;
        
        %t = (10^-1)^(-1/length(estn)) - 1
        
        imgf(j:rightIdx,i:rightB) = (data(j:rightIdx,i:rightB)-mn)/sd;
        %imgf(j:rightIdx,i:rightB) = (data(j:rightIdx,i:rightB)-t*sm)/sd;
        imgf(find(imgf<0)) = 0;
    end
    %break;
end
end