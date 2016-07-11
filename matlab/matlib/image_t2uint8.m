function img=image_t2uint8(m)
raw=typecast(m.I,'uint8');
if m.c==1
  img=reshape(raw,m.w,m.h)';
elseif m.c==3
  imgb=reshape(raw(1:3:end),m.w,m.h)';
  imgg=reshape(raw(2:3:end),m.w,m.h)';
  imgr=reshape(raw(3:3:end),m.w,m.h)';
  img=cat(3,imgr,imgg,imgb);
else
  error('wrong input type!');
end
end