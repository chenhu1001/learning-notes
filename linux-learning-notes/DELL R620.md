DELL R620服务器配置

# 1、初始化网络
编辑/etc/sysconfig/network-scripts下的em1、em2、em3和em4,修改最后一行ONBOOT=yes，最后重启网络service network restart

# 2、安装Docker
```
curl -sSL https://get.daocloud.io/docker | sh
service docker restart
systemctl enable docker
```

# 3、安装Portainer
```
docker run -d -p 9000:9000 -v /var/run/docker.sock:/var/run/docker.sock --restart=always --name portainer portainer/portainer
```

# 4、安装jdk1.8
* （1）、将下载好的安装包jdk-8u211-linux-x64.tar.gz上传到服务器指定目录下
* （2）、解压安装包：tar -zxvf jdk-8u211-linux-x64.tar.gz
* （3）、复制安装包到/usr/local/java/目录下：cp -r jdk1.8.0_211/ /usr/local/java
* （4）、添加环境变量：编辑/etc/profile文件：vi /etc/profile，在最后面添加：
```
JAVA_HOME=/usr/local/java
CLASSPATH=$JAVA_HOME/lib/
PATH=$PATH:$JAVA_HOME/bin
export PATH JAVA_HOME CLASSPATH
```
* （5）、重新加载/etc/profile文件source /etc/profile 
* （6）、使用java -version命令验证jdk是否安装成功:
```
[root@localhost java]# java -version
java version "1.8.0_211"
Java(TM) SE Runtime Environment (build 1.8.0_211-b12)
Java HotSpot(TM) 64-Bit Server VM (build 25.211-b12, mixed mode)
```

# 5、运行investool
```
nohup ./investool webserver &
```

# 6、运行frp
```
nohup ./frpc  -c frpc_full.ini &
```

# 7、运行CloudflareST
```
./CloudflareST
```

# 8、开防火墙端口
```
firewall-cmd --zone=public --add-port=9000/tcp --permanent
firewall-cmd --zone=public --add-port=4869/tcp --permanent
systemctl restart firewalld.service
```

# 9、安装git、telnet
```
yum install -y git
yum install -y telnet
```

# 10、安装maven
```
yum install -y maven
```

# 11、安装docker-compose
```
curl -L https://get.daocloud.io/docker/compose/releases/download/1.21.2/docker-compose-`uname -s`-`uname -m` > /usr/local/bin/docker-compose
chmod +x /usr/local/bin/docker-compose
docker-compose -v
```

# 12、搭建环境
Rocketmq需要特殊配制一下节点ip，在config/broker.conf文件中，将brokerIP1修改为部署docker的局域网ip
```
docker-compose up -d
docker ps
```

```
firewall-cmd --zone=public --add-port=3306/tcp --permanent
firewall-cmd --zone=public --add-port=9001/tcp --permanent
systemctl restart firewalld.service
```

# 13、安装NodeJS
* 首先安装wget
```
yum install -y wget
```
* 下载nodejs最新的bin包
```
wget https://nodejs.org/dist/v12.18.3/node-v12.18.3-linux-x64.tar.xz
```
* 然后就是等着下载完毕。
依次执行
```
xz -d node-v12.18.3-linux-x64.tar.xz
tar -xf node-v12.18.3-linux-x64.tar
ln -s /root/soft/node-v12.18.3-linux-x64/bin/node /usr/bin/node
ln -s /root/soft/node-v12.18.3-linux-x64/bin/npm /usr/bin/npm
ln -s /root/soft/node-v12.18.3-linux-x64/bin/npm /usr/bin/npx
npm install -g yarn
ln -s /root/soft/node-v12.18.3-linux-x64/bin/yarn /usr/bin/yarn
node -v
```

# 14、启动Nacos
 ```
 sh startup.sh -m standalone
 ```

# 15、安装Zookeeper
```
docker run -d -e TZ="Asia/Shanghai" -p 2181:2181 -v $PWD/data:/data --name zookeeper --restart always zookeeper
```

# 16、安装mongo
```
docker run -itd --name mongo --restart=always -p 27017:27017 mongo
```
