# 1、首先需要安装docker和docker-compose
curl -fsSL https://get.docker.com -o get-docker.sh  && bash get-docker.sh
systemctl enable docker
systemctl restart docker

sudo curl -L "https://github.com/docker/compose/releases/download/1.24.1/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
sudo chmod +x /usr/local/bin/docker-compose

# 2、拷贝nginx配置文件
使用 nginx 都躲不了修改配置文件，不管是反向代理博客，或者是反向代理其他都需要进行修改配置，那么将 nginx 配置文件映射到宿主机是很有必要的。
```
# 创建目录
mkdir -p /home/docker/nginx && \
cd /home/docker/nginx && \
mkdir conf.d && \
mkdir html && \
mkdir logs
 
# 不映射文件启动容器，然后 使用 docker cp 拷贝配置文件
docker run -d --name nginx -p 88:80 nginx && \
docker cp nginx:/etc/nginx/nginx.conf ./ && \
docker cp nginx:/etc/nginx/conf.d/default.conf ./conf.d/ && \
docker cp nginx:/usr/share/nginx/html/index.html ./html/ && \
docker rm -f nginx
```
修改nginx.conf在http下面加上：client_max_body_size 1024M;

# 3、在/home/docker/wordpress目录下新建 docker-compose.yaml 文件
```
version: '3.1'
services:
  nginx:
    image: nginx
    container_name: nginx
    restart: always
    network_mode: "host"
    volumes:
      - /home/docker/nginx/nginx.conf:/etc/nginx/nginx.conf
      - /home/docker/nginx/conf.d:/etc/nginx/conf.d
      - /home/docker/nginx/logs:/var/log/nginx
      - /home/docker/nginx/www:/www
      - /etc/localtime:/etc/localtime:ro
  blog:
    image: wordpress:6.0.3
    container_name: blog
    restart: always
    ports:
      - 30080:80
    environment:
      WORDPRESS_DB_HOST: mysql
      WORDPRESS_DB_USER: root
      WORDPRESS_DB_PASSWORD: admin@123!
      WORDPRESS_DB_NAME: blog
    volumes:
      - /home/docker/blog:/var/www/html
      - ./uploads.ini:/usr/local/etc/php/conf.d/uploads.ini
    depends_on:
     - mysql
  mysql:
   image: mysql:8.0.26
   container_name: mysql
   restart: always
   ports:
     - 30001:3306
   environment:
     MYSQL_DATABASE: blog
     MYSQL_ROOT_PASSWORD: admin@123!
   volumes:
     - /home/docker/mysql/data:/var/lib/mysql
     - /home/docker/mysql/init:/docker-entrypoint-initdb.d
```

在/home/docker/wordpress目录下创建文件uploads.ini：
```
file_uploads = On
memory_limit = 500M
upload_max_filesize = 500M
post_max_size = 500M
max_execution_time = 600
```
然后运行 docker-compose up -d 启动容器。
# 4、修改nginx配置文件
http
```
cat > /home/docker/nginx/conf.d/default.conf <<EOF
server {
    listen 80;
    server_name test.com www.test.com;
    access_log  /var/log/nginx/blog.access.log  main;
    error_log  /var/log/nginx/blog.error.log notice;
 
    location ~* /xmlrpc.php {
      deny all;
    }
 
    location / {
      add_header 'Access-Control-Allow-Origin' '*';
      add_header 'Access-Control-Allow-Credentials' 'true';
      add_header 'Access-Control-Allow-Methods' 'GET, POST, OPTIONS';
      add_header 'Access-Control-Allow-Headers' 'DNT,X-CustomHeader,Keep-Alive,User-Agent,X-Requested-With,If-Modified-Since,Cache-Control,Content-Type';
      proxy_set_header X-Real-IP \$remote_addr;
      proxy_set_header Host \$host;
      proxy_set_header X-Forwarded-For \$proxy_add_x_forwarded_for;
      proxy_pass http://localhost:30080/;
    }
}
EOF
```
https
```
cat > /home/docker/nginx/conf.d/default.conf <<EOF
server {
    listen       443 ssl;
    listen  [::]:443 ssl;
    server_name test.com www.test.com;
    ssl_certificate  /etc/nginx/conf.d/cert/test.com/test.cer;
    ssl_certificate_key /etc/nginx/conf.d/cert/test.com/test.com.key;
    access_log  /var/log/nginx/blog.access.log  main;
    error_log  /var/log/nginx/blog.error.log notice;
 
    location ~* /xmlrpc.php {
      deny all;
    }
 
    location / {
      add_header 'Access-Control-Allow-Origin' '*';
      add_header 'Access-Control-Allow-Credentials' 'true';
      add_header 'Access-Control-Allow-Methods' 'GET, POST, OPTIONS';
      add_header 'Access-Control-Allow-Headers' 'DNT,X-CustomHeader,Keep-Alive,User-Agent,X-Requested-With,If-Modified-Since,Cache-Control,Content-Type';
      add_header Content-Security-Policy upgrade-insecure-requests;
      proxy_set_header X-Real-IP \$remote_addr;
      proxy_set_header Host \$host;
      proxy_set_header X-Forwarded-For \$proxy_add_x_forwarded_for;
      proxy_pass http://localhost:30080/;
    }
}
server {
    listen 80;
    server_name test.com www.test.com;
    rewrite ^(.*)$ https://\$host:443\$1 permanent;
}
```
需要说明的是，配置 https 需要申请 ssl 证书，下载后放在指定位置，然后配置 ssl_certificate 和 ssl_certificate_key。

此外，由于配置了 rewrite，访问 80 端口会强制跳转到 443。

配置完成后需要重启 nginx。
```
docker restart nginx
```
