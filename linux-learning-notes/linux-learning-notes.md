# linux-learning-notes
Linux学习笔记
## 常用
```
// 查看开机启动项
systemctl list-unit-files |grep enabled

// 设置防火墙开机启动
# systemctl enable firewalld

// 开端口命令
firewall-cmd --zone=public --add-port=80/tcp --permanent

// 重启防火墙
systemctl restart firewalld.service

// 文件夹大小查询
du -h --max-depth=1
```

## kcptun

```
wget --no-check-certificate https://github.com/kuoruan/shell-scripts/raw/master/kcptun/kcptun.sh
chmod +x ./kcptun.sh
./kcptun.sh
```
强制kill掉某端口
```
lsof -i :9000 | grep LISTEN | awk '{print $2}' | xargs kill -9
```
杀掉某个应用
```
ps -ef|grep -v grep|grep comprehensive-backup-service|awk '{print $2}' | xargs kill -9
```

## live555

```
wget  http://www.live555.com/liveMedia/public/live555-latest.tar.gz
tar xzf live555-latest.tar.gz
cd live
./genMakefiles linux-64bit    // 注意后面这个参数是根据当前文件夹下config.<后缀>获取得到的
make && make install          // 在usr/local/include出现四个文件夹的头文件、/usr/local/lib下出现链接库
```

## yum命令

```
yum -y install wget   // 安装wget
yum -y install git    // 安装git
yum -y list lsof      // 列出包含lsof的软件包
yum -y install lsof   // 安装lsof
```

## netstat

```
netstat -ntlp     // 查看服务器运行的进程服务和监听端口
```

## lsof

```
lsof -i :8989     // 查看8989端口现在运行的情况
```

## java

```
yum -y list java*
yum -y install java-1.8.0-openjdk.x86_64
```

## go

```
yum -y list golang
yum -y install golang.x86_64
```

## redis

```
yum install redis
systemctl start redis.service
systemctl enable redis.service
```

* 设置redis密码  
  打开文件/etc/redis.conf，找到其中的#requirepass foobared，去掉前面的#，并把foobared改成你的密码。

* 设置远程访问
  注释#bind 127.0.0.1

* 测试redis连接是否成功
  redis-cli -h 198.xxx.xxx.xxx -p 6379

离线安装
```
$ wget http://download.redis.io/releases/redis-4.0.9.tar.gz
$ tar xzf redis-4.0.9.tar.gz
$ cd redis-4.0.9
$ make
```
The binaries that are now compiled are available in the src directory. Run Redis with:
```
src/redis-server
```
常用命令：
```
AUTH 123456; // 鉴权
EXISTS key; // 检查key是否存在
SET key value; // 设置一个键的值
GET key; // 获取一个建的值
DEL key; // 删除键对
```

## frp

```
# frpc.ini
[common]
server_addr = 198.181.46.166
server_port = 5443
privilege_token = tIMrKiL94XZeXhb3

[web]
privilege_mode = true
type = http
local_port = 4000
custom_domains = www.clang.online
```

端口范围
```
# frpc.ini
[common]
server_addr = 198.181.46.166
server_port = 5443
privilege_token = tIMrKiL94XZeXhb3

[range:test_tcp]
type = tcp
local_ip = 127.0.0.1
local_port = 80,443
remote_port = 4080,8443
```

## MySQL数据库远程访问权限如何打开
### 1、改表法
可能是你的帐号不允许从远程登陆，只能在localhost。这个时候只要在localhost的那台电脑，登入mysql后，更改 "mysql" 数据库里的 "user" 表里的 "host" 项，从"localhost"改称"%"

```
mysql -u root -p 
mysql>use mysql; 
mysql>update user set host = '%' where user = 'root'; 
mysql>select host, user from user;

``` 
### 2、授权法
在安装mysql的机器上运行：

```
// 这样应该可以进入MySQL服务器
mysql -h localhost -u root
```

```
// 赋予任何主机访问数据的权限
mysql>GRANT ALL PRIVILEGES ON *.* TO 'root'@'%'WITH GRANT OPTION
```

例如，你想myuser使用mypassword从任何主机连接到mysql服务器的话。

```
mysql>GRANT ALL PRIVILEGES ON *.* TO 'myuser'@'%'IDENTIFIED BY 'mypassword' WITH GRANT OPTION; 
```

如果你想允许用户myuser从ip为192.168.1.6的主机连接到mysql服务器，并使用mypassword作为密码

```
mysql>GRANT ALL PRIVILEGES ON *.* TO 'myuser'@'192.168.1.3'IDENTIFIED BY 'mypassword' WITH GRANT OPTION;
```

```
// 修改生效
mysql>FLUSH PRIVILEGES
```

```
// 退出MySQL服务器，这样就可以在其它任何的主机上以root身份登录
mysql>EXIT
```

## 防火墙配置
### firewall

```
# service firewalld status   // 查看防火墙状态（disabled 表明 已经禁止开启启动 enable 表示开机自启，inactive 表示防火墙关闭状态 activated（running）表示为开启状态）
# service firewalld start;  或者 #systemctl start firewalld.service;#开启防火墙
# service firewalld stop;  或者 #systemctl stop firewalld.service;#关闭防火墙
# service firewalld restart;  或者 #systemctl restart firewalld.service;  #重启防火墙
# systemctl disable firewalld.service   // 禁止防火墙开启自启
# systemctl enable firewalld   // 设置防火墙开机启动
# yum remove firewalld   // #卸载firewall
```

### 3、其他
```
mysql> GRANT ALL PRIVILEGES ON *.* TO root @'%' IDENTIFIED BY "123456";
Query OK, 0 rows affected (0.04 sec)
 
mysql> FLUSH PRIVILEGES;
Query OK, 0 rows affected (0.02 sec)
 
mysql> 
```

## iptables

```
#yum install iptables-services   // 安装iptables防火墙
#vi /etc/sysconfig/iptables   // 编辑防火墙配置文件，开放3306端口
添加配置：-A INPUT -p tcp -m state --state NEW -m tcp --dport 3306 -j ACCEPT
#systemctl restart iptables.service   // 最后重启防火墙使配置生效
#systemctl enable iptables.service   // 设置防火墙开机启动
```

## CentOS 7 使用iptables 开放端口
CentOS 7.0默认使用的是firewall作为防火墙，这里改为iptables防火墙。
1、关闭firewall：

```
systemctl stop firewalld.service
systemctl disable firewalld.service
systemctl mask firewalld.service
```

2、安装iptables防火墙

```
yum install iptables-services -y
```

3.启动设置防火墙

```
# systemctl enable iptables
# systemctl start iptables
```

4.查看防火墙状态

```
systemctl status iptables
```

5编辑防火墙，增加端口

```
vi /etc/sysconfig/iptables #编辑防火墙配置文件
-A INPUT -m state --state NEW -m tcp -p tcp --dport 22 -j ACCEPT
-A INPUT -m state --state NEW -m tcp -p tcp --dport 80 -j ACCEPT
-A INPUT -m state --state NEW -m tcp -p tcp --dport 3306 -j ACCEPT
:wq! #保存退出
```

3.重启配置，重启系统

```
systemctl restart iptables.service #重启防火墙使配置生效
systemctl enable iptables.service #设置防火墙开机启动
```

## Nginx将静态文件响应POST请求，提示405错误问题
在nginx.conf中，请求的静态数据路径中，添加如下语句error_page 405=200 $request_uri：
```
location ~ \.(action|jsp) {
    root $testDataFold;
    error_page 405 =200 $request_uri;
}
```
## Centos使用yum构建离线安装包
在Centos中我们安装软件的方式往往是使用yum进行在线安装，但是在某些情况下（公司服务器使用的是内网）我们就需要使用yum将安装包下载到本地，并进行离线安装。以下是离线安装docker步骤：

首先创建一个目录用于存放安装包：

```
mkdir /usr/local/docker
```

通过yum命令下载docker安装文件：

```
yum install --downloadonly --downloaddir=/usr/local/docker docker-io
```

使用rpm命令进行安装
```
rpm -Uvh --force --nodeps /usr/local/docker/*rpm
```

## 安装sz,rz命令
```
wget https://raw.githubusercontent.com/chenhu1001/linux-learning-notes/master/rzsz-3.48.tar.gz
tar zxvf  rzsz-3.34.tar.gz
cd src
// 修改Makefile文件第四行的"OFLAG= -O"为"OFLAG= -O -DREGISTERED"
make posix
cp rz sz /usr/bin
```

## 自动上传文件脚本
```
#!/bin/bash

#---------------------------------------------------------------------------------------------
dir="/Users/chenhu/Documents/ticketPlatform2/ticketManagementH5/target/"  #jar 生成路径
#---------------------------------------------------------------------------------------------

main(){
	echo "...............................jar包开始上传到服务器"
	expect -c "
	spawn scp $dir/ticketManagementH5.jar  root@192.168.129.16:/data/h5/ticketManagementH5.jar
	expect {
    \"*assword:\" {set timeout 300; send \"123456\r\";}
      }
	expect eof"
	echo -e "==>jar包上传到服务器完成"
}

main
```


## Maven自动打包
```
cd /Users/chenhu/Documents/ticketPlatform2/ticketManagementH5
/Users/chenhu/apache-maven-3.5.4/bin/mvn clean install
```

## 远程部署服务
```
#!/bin/bash

#查询项目进程
JAR_pid=`ssh root@192.168.129.16 ps -ef | grep "ticketManagementH5.jar" | grep -v "grep"|awk '{print $2}'`
echo "JAR_pid="
echo $JAR_pid
if [  "$JAR_pid" != "" ];then
    ssh root@192.168.129.16 kill -9 $JAR_pid     #杀死项目进程
    echo "杀死项目进程"
else
    echo "进程不存在可以继续部署"
fi

echo "...............................开始启动服务"
ssh root@192.168.129.16 "nohup /data/tools/jdk1.8.0_151/bin/java -jar /data/h5/ticketManagementH5.jar &"
echo -e "==>服务器部署完成"
```

## 安装nginx
安装yum-utils
```
yum install yum-utils
```
设置 yum仓库, 编辑 /etc/yum.repos.d/nginx.repo
```
[nginx-stable]
name=nginx stable repo
baseurl=http://nginx.org/packages/centos/$releasever/$basearch/
gpgcheck=1
enabled=1
gpgkey=https://nginx.org/keys/nginx_signing.key

[nginx-mainline]
name=nginx mainline repo
baseurl=http://nginx.org/packages/mainline/centos/$releasever/$basearch/
gpgcheck=1
enabled=0
gpgkey=https://nginx.org/keys/nginx_signing.key
```
默认使用 stable的nginx, 要使用最新的需要切换到mainline (可选)
```
yum-config-manager --enable nginx-mainline
```
安装nginx
```
yum install nginx
```

## centos7防火墙关闭以及清除防火墙规则
关闭防火墙
```
查看防火墙状态
firewall-cmd --state

停止firewall
systemctl stop firewalld.service

禁止firewall开机启动
systemctl disable firewalld.service 
```
清除防火墙规则
```
iptables -F 
(flush 清除所有的已定规则)

iptables -X 
(delete 删除所有用户“自定义”的链（tables）)

iptables -Z 
（zero 将所有的chain的计数与流量统计都归零）
```

## 同步时间
```
yum install ntpdate -y
ntpdate 0.asia.pool.ntp.org
```

可以加入到定时任务中

```
* */1 * * * /usr/sbin/ntpdate pool.ntp.org
```

## 安装zookeeper
```
tar -zxvf zookeeper-3.4.6.tar.gz
mkdir -p /data/zookeeper
cp zoo_sample.cfg zoo.cfg

// 把dataDir的路径修改成刚刚自己创建的data路径
vi zoo.cfg

cd ../bin/
./zkServer.sh start

// 查看是否启动成功
./zkServer.sh status
```

## CentOS7防火墙常用操作

```
// 查看防火墙状态
firewall-cmd --state

// 开启防火墙
systemctl start firewalld.service

// 设置开机自启
systemctl enable firewalld.service

// 重启防火墙
systemctl restart firewalld.service

// 检查防火墙状态是否打开
firewall-cmd --state

// 查看防火墙设置开机自启是否成功
systemctl is-enabled firewalld.service;echo $?

// 开端口命令
firewall-cmd --zone=public --add-port=80/tcp --permanent

// 查看开启的所有端口
firewall-cmd --list-ports
```

## screen安装
```
CentOS
yum install screen

Debian/Ubuntu
apt-get install screen

1、创建screen会话
screen -S lnmp(会话名)

2、暂时离开，保留screen会话中的任务或程序
当需要临时离开时（会话中的程序不会关闭，仍在运行）可以用快捷键Ctrl+a d(即按住Ctrl，依次再按a,d)

3、恢复screen会话
当回来时可以再执行执行：screen -r lnmp 即可恢复到离开前创建的lnmp会话的工作界面。如果忘记了，或者当时没有指定会话名，可以执行：screen -ls screen会列出当前存在的会话列表。

4、关闭screen的会话
执行：exit，会提示：[screen is terminating]，表示已经成功退出screen会话

5、远程演示
首先演示者先在服务器上执行 screen -S test 创建一个screen会话，观众可以链接到远程服务器上执行screen -x test 观众屏幕上就会出现和演示者同步。

6、常用快捷键
Ctrl+a c ：在当前screen会话中创建窗口
Ctrl+a w ：窗口列表
Ctrl+a n ：下一个窗口
Ctrl+a p ：上一个窗口
Ctrl+a 0-9 ：在第0个窗口和第9个窗口之间切换
```

## shell脚本创建用户
```
#!/bin/sh

#设置变量name接收第一个参数（要创建的用户名），$n表示第n个参数，且=两边不能有空格
name=$1
#设置变量pass接收第二个参数（要为其设置的密码）
pass=$2

#echo语句会输出到控制台，${变量}或者 $变量 表示变量代表的字符串
echo "you are setting username : ${name}"
echo "you are setting password : $pass for ${name}"

#添加用户$name，此处sudo需要设置为无密码，后面将会作出说明
sudo useradd $name

#如果上一个命令正常运行，则输出成功，否则提示失败并以非正常状态退出程序
# $?表示上一个命令的执行状态，-eq表示等于，[ 也是一个命令
# if fi 是成对使用的，后面是前面的倒置，很多这样的用法。
if [ $? -eq 0 ];then
   echo "user ${name} is created success!"
else
   echo "user ${name} is created failed!!!"
   exit 1
fi
#sudo passwd $name会要求填入密码，下面将$pass作为密码传入
#echo $pass | sudo passwd $name --stdin  &> /dev/null

#ubuntu系统不支持passwd的stdin参数，所以要使用chpasswd命令
sudo echo "$name:$pass" | chpasswd
if [ $? -eq 0 ];then
   echo "${name}'s password is set success!"
else
   echo "${name}'s password is set failed!!!"
fi

if [ -d /home/$name ]
then
   echo "/home/$name is already exist!"
   exit 1
else
    sudo mkdir /home/$name
    sudo chown -R $name /home/$name
fi
```

## Ubuntu报“xxx is not in the sudoers file.This incident will be reported” 错误解决方法
```
1.切换到root用户下

2./etc/sudoers文件默认是只读的，对root来说也是，因此需先添加sudoers文件的写权限,命令是:
chmod u+w /etc/sudoers

3.编辑sudoers文件
vi /etc/sudoers
找到这行 root ALL=(ALL) ALL,在他下面添加xxx ALL=(ALL) ALL (这里的xxx是你的用户名)

ps:这里说下你可以sudoers添加下面四行中任意一条
youuser            ALL=(ALL)                ALL
%youuser          ALL=(ALL)                ALL
youuser            ALL=(ALL)                NOPASSWD: ALL
%youuser          ALL=(ALL)                NOPASSWD: ALL

第一行:允许用户youuser执行sudo命令(需要输入密码).
第二行:允许用户组youuser里面的用户执行sudo命令(需要输入密码).
第三行:允许用户youuser执行sudo命令,并且在执行的时候不输入密码.
第四行:允许用户组youuser里面的用户执行sudo命令,并且在执行的时候不输入密码.

4.撤销sudoers文件写权限,命令:
chmod u-w /etc/sudoers
```

## 解决Screen出现Cannot open your terminal ‘/dev/pts/0’问题
```
切换用户后，执行下列语句即可：
script /dev/null
```

## CentOS 8 开启 BBR/升级内核

```
echo "net.core.default_qdisc=fq" >> /etc/sysctl.conf
echo "net.ipv4.tcp_congestion_control=bbr" >> /etc/sysctl.conf
sysctl -p
// 重启 如果输出包含 BBR，说明启用成功
sysctl -n net.ipv4.tcp_congestion_control
lsmod | grep bbr
```
## 一个tomcat设置多个端口，多个端口对应多个应用
修改tomcat/conf目录里面server.xml文件  
例如下面这样新增一个8090端口，设置下appBase目录，这样就可以用一个tomcat监听多个端口，每个端口都可以放应用了。我这样新增下面这个配置以后，tomcat就监听了2个端口（8080，8090）  
webapps目录和mywebapps目录的应用都会启动，可以根据不同的端口进行访问里面的应用
```
<?xml version="1.0" encoding="UTF-8"?>

<Server port="8005" shutdown="SHUTDOWN">
  <Listener className="org.apache.catalina.startup.VersionLoggerListener" />
  <Listener className="org.apache.catalina.core.AprLifecycleListener" SSLEngine="on" />
  <Listener className="org.apache.catalina.core.JreMemoryLeakPreventionListener" />
  <Listener className="org.apache.catalina.mbeans.GlobalResourcesLifecycleListener" />
  <Listener className="org.apache.catalina.core.ThreadLocalLeakPreventionListener" />

  <GlobalNamingResources>

    <Resource name="UserDatabase" auth="Container"
              type="org.apache.catalina.UserDatabase"
              description="User database that can be updated and saved"
              factory="org.apache.catalina.users.MemoryUserDatabaseFactory"
              pathname="conf/tomcat-users.xml" />
  </GlobalNamingResources>


  <Service name="Catalina">
 
    <Connector port="80" protocol="HTTP/1.1"
               connectionTimeout="20000"
               redirectPort="8443" />
     
    <Connector port="8009" protocol="AJP/1.3" redirectPort="8443" />
 
    <Engine name="Catalina" defaultHost="localhost">
 
      <Realm className="org.apache.catalina.realm.LockOutRealm">
 
        <Realm className="org.apache.catalina.realm.UserDatabaseRealm"
               resourceName="UserDatabase"/>
      </Realm>

      <Host name="localhost"  appBase="webapps"
            unpackWARs="true" autoDeploy="true">
 
        <Valve className="org.apache.catalina.valves.AccessLogValve" directory="logs"
               prefix="localhost_access_log" suffix=".txt"
               pattern="%h %l %u %t &quot;%r&quot; %s %b" />

      </Host>
    </Engine>
  </Service>
  
  <Service name="Catalina2">
 
    <Connector port="81" protocol="HTTP/1.1"
               connectionTimeout="20000"
               redirectPort="8445" />
     
    <Connector port="8010" protocol="AJP/1.3" redirectPort="8445" />
 
    <Engine name="Catalina" defaultHost="localhost">
 
      <Realm className="org.apache.catalina.realm.LockOutRealm">
 
        <Realm className="org.apache.catalina.realm.UserDatabaseRealm"
               resourceName="UserDatabase"/>
      </Realm>

      <Host name="localhost"  appBase="webapps2"
            unpackWARs="true" autoDeploy="true">
 
        <Valve className="org.apache.catalina.valves.AccessLogValve" directory="logs"
               prefix="localhost_access_log" suffix=".txt"
               pattern="%h %l %u %t &quot;%r&quot; %s %b" />

      </Host>
    </Engine>
  </Service>
</Server>
```
## 修改oracle密码
步骤1：使用SSH工具已root身份连接服务器；  
步骤2：切换到oracle 用户（linux上的用户）： su - oracle  
步骤3：使用sqlplus并以管理员的身份登录: sqlplus / as sysdba （出现SQL>即说明登陆成功）  
步骤4：在SQL>后面输入：alter user 数据库连接用户名 identified by "新密码";（注：1.分号不能忘；2.密码若为数字和字母组合，且数字开头，密码要用双引号）  
步骤5：修改成功。

## pip install sasl报错fatal error: sasl/sasl.h: No such file or directory
Ubuntu系统的话可能需要先装好libsasl2-dev
CentOS系统需要预先装好python-devel和cyrus-sasl-devel

## zip分卷、合卷
```
zip google.zip Google_Maps_API.doc // 先压缩不然就报上面的错
zip -s 4m google.zip --out ziptest   // 分卷
cat ziptest.z* > google_bak.zip // 利用cat来合卷
unzip google_bak.zip  //解压  
```

## 数据库备份
```
#!/bin/bash
#要备份网站的数据库信息
mysql_user="root"
mysql_pwd="123456"
dbname="dbname"
#定义备份邮件标题，可以区分是哪一天的
d=$(date +"%d-%m-%Y-%H:%M:%S")
#发送邮件标题和内容，这个是次要的，主要是附件中的数据库
t="databases backup - $d"
#数据库文件名称
BackName=backup_mysql_$d
#备份数据库存放路径<可以写上定期删除的脚本，我这里没有写>
BackPath=/root/DBbackup/
#导出数据库的所有库
mysqldump -u${mysql_user} -p${mysql_pwd} $dbname > $BackPath$BackName.sql
#用mutt发送邮件，发送到我们指定的邮箱里"backup@itbulu.com"         -s  邮件标题  -a  指定的附件<多个的话后面继续加-a>
echo "$t"|mutt -s "$t" 123456789@qq.com -a $BackPath$BackName.sql
exit

00 03 * * * /usr/bin/sh /root/DBbackup/backup.sh
```

## 定时清理日志文件
```
#! /bin/bash
if [[ $1 && $2 && -d $1 ]]; then
	# 时间范围
	saveTime=$[$2*24*60*60]
	function clearLog(){
		for i in `ls $1`; do
			if [[ -f $1/$i ]]; then
				# 获取文件修改时间
				modifiedTime=`stat -c %Y $1/$i`
				# 获取系统当前时间
				currTime=`date +%s`
				if [[ $[ $currTime - $modifiedTime] -gt $saveTime ]]; then
					rm -f $1/$i
				fi

			elif [[ -d $1/$i ]]; then
				clearLog $1/$i
			fi
		done
	}

	clearLog $1

	echo '清理成功'
else
	echo '请输入有效的文件目录和天数'
	exit
fi
```
该脚本需要指定两个参数，第一个参数是log所在的根目录，第二个参数是需要清理前几天的数据。 例:
```
echo '0 0 1 * * /bin/sh /your/save/path/clear.sh /your/log/path 15' > /your/crontab/file
```
把执行命令放入到你的crontab队列中，每月1号零点清理一次，会把log的最后修改时间距离现在已经超过15天的文件删除，支持递归删除。

## zip命令
```
将 /home/html/ 这个目录下所有文件和文件夹打包为当前目录下的 html.zip：
zip -q -r html.zip /home/html

如果在我们在 /home/html 目录下，可以执行以下命令：
zip -q -r html.zip *
```

## 网络连通性
```
nc -z xxx.xxx.xxx.xxx 80
```

## Windows命令
```
// 刷新DNS缓存
ipconfig /flushdns

// 删除目录
rmdir /s/q public

// 复制文件
copy /y sitemap.xml public\sitemap.xml
```

## 检测网站是否被检索
```
site: xxx.github.io
```

## Java程序挂掉自动重启
```
#!/bin/sh
 
#切换到目录
cd /root
date=`date`
 
#查询程序是否运行
ps -ef|grep -v grep|grep fund-0.0.1-SNAPSHOT.jar
 
# $? -ne 0 不存在 $? -eq 0存在 
if [ $? -ne 0 ]
then
    nohup java -jar fund-0.0.1-SNAPSHOT.jar >/dev/null 2>&1 &
    echo $date  ":=============== restart ===============" >> monitoring.log
else
    echo $date  ":=============== is normal ==============" >> monitoring.log
fi
```
```
* * * * * /usr/bin/sh /root/monitoring.sh
```

## 安装 Supervisor
在 CentOS 7 上安装 Supervisor 通常可以通过以下步骤完成：

1. **使用 yum 安装 Supervisor：**

   ```bash
   sudo yum install epel-release
   sudo yum install supervisor
   ```

   第一行命令安装 `epel-release` 以获取额外的软件包，第二行命令安装 Supervisor。

2. **启动 Supervisor 服务：**

   ```bash
   sudo systemctl start supervisord
   ```

3. **设置 Supervisor 开机自启动：**

   ```bash
   sudo systemctl enable supervisord
   ```

   这样 Supervisor 将在系统启动时自动启动。

4. **检查 Supervisor 服务状态：**

   ```bash
   sudo systemctl status supervisord
   ```

   确保服务已经正常运行。

5. **连接到 `supervisorctl` 控制台：**

   ```bash
   sudo supervisorctl
   ```

   这会将你连接到 `supervisorctl` 控制台，你可以在这里执行相关命令。

6. **退出 `supervisorctl` 控制台：**

   ```bash
   quit
   ```

   或者使用快捷键 `Ctrl + D` 退出。

现在，Supervisor 已经安装并正在运行，你可以使用 `supervisorctl` 来管理你的进程。请根据你的需要配置和添加需要由 Supervisor 管理的进程。

## supervisorctl介绍和使用
Supervisorctl 是 Supervisor 的管理工具，用于控制 Supervisor 管理的进程。Supervisor 是一个进程管理器，可以用于启动、停止、重启、查看进程状态等。

**supervisorctl 的使用**

supervisorctl 是一个命令行工具，可以通过以下方式启动：

```
sudo supervisorctl
```

启动后，supervisorctl 会进入交互模式，可以通过以下命令进行操作：

* **start**：启动进程
* **stop**：停止进程
* **restart**：重启进程
* **status**：查看进程状态
* **log**：查看进程日志
* **tail**：实时查看进程日志
* **reload**：重新加载配置文件
* **update**：根据最新配置启动或重启进程

**示例**

以下是一些 supervisorctl 的使用示例：

* 启动名为 `my_app` 的进程：

```
sudo supervisorctl start my_app
```

* 停止名为 `my_app` 的进程：

```
sudo supervisorctl stop my_app
```

* 重启名为 `my_app` 的进程：

```
sudo supervisorctl restart my_app
```

* 查看名为 `my_app` 的进程状态：

```
sudo supervisorctl status my_app
```

* 查看所有进程的状态：

```
sudo supervisorctl status
```

* 查看名为 `my_app` 的进程日志：

```
sudo supervisorctl log my_app
```

* 实时查看名为 `my_app` 的进程日志：

```
sudo supervisorctl tail my_app
```

* 重新加载配置文件：

```
sudo supervisorctl reload
```

* 根据最新配置启动或重启进程：

```
sudo supervisorctl update
```

**supervisorctl 的参数**

supervisorctl 还支持一些参数，可以通过 `-h` 或 `--help` 查看帮助信息。

以下是一些常用的参数：

* **-c**：指定配置文件路径
* **-u**：指定用户名
* **-p**：指定密码
* **-s**：指定 Supervisor 的地址和端口

**supervisorctl 的安全性**

Supervisor 默认不支持认证，因此 supervisorctl 可以通过任何用户访问。如果需要提高安全性，可以通过配置文件中的 `[inet_http_server]` 段来启用认证。

**supervisorctl 的其他功能**

除了上述功能之外，supervisorctl 还支持一些其他功能，例如：

* 创建、删除、修改进程配置
* 设置进程的启动参数、日志文件路径等
* 查看进程的资源使用情况
* 发送信号给进程

**总结**

supervisorctl 是一个功能强大的进程管理工具，可以用于管理 Supervisor 管理的进程。通过 supervisorctl，可以轻松地启动、停止、重启、查看进程状态等。

## 配置supervisord应用配置文件
**编辑配置文件**
应用的配置文件在：/etc/supervisord.d目录
可以通过编辑配置文件来创建进程配置。Supervisor 的配置文件采用 INI 格式，可以使用任何文本编辑器进行编辑。

要创建一个新的进程配置，需要在配置文件中添加一个新的 `[program:<program_name>]` 段。`<program_name>` 是进程的名称。

例如，要创建一个名为 `my_app` 的进程，可在配置文件中添加以下内容：

```
[program:my_app]
command=/opt/my_app/my_app.py
directory=/opt/my_app
autostart=true
autorestart=true
```

这段配置将创建一个名为 `my_app` 的进程，该进程将启动 `/opt/my_app/my_app.py` 脚本。进程将自动启动并在发生错误时自动重启。
配置完成后，重启supervisord
```
sudo systemctl restart supervisord
```

**进程配置的详细说明**

进程配置的详细说明可以参考 Supervisor 的文档。

以下是一些常用的进程配置选项：

* **command**：指定进程的可执行文件或脚本。
* **directory**：指定进程的启动目录。
* **autostart**：指定进程是否自动启动。
* **autorestart**：指定进程是否自动重启。
* **exitcodes**：指定进程的退出码。
* **logfile**：指定进程的日志文件路径。
* **priority**：指定进程的优先级。

通过配置这些选项，可以对进程进行细粒度的控制。
