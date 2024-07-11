# java-learning-notes
## 1、java命令
* javac：用于将java源文件编译为class字节码文件
* java：可以运行class字节码文件，如：java HelloWorld

## 2、java分为三个体系
* JavaSE（J2SE）java平台标准版
* JavaEE（J2EE）java平台企业版
* JavaME（J2ME）java平台微型版

## 3、java语言特性
java语言支持动态绑定，而C++只对虚函数使用动态绑定。

## 4、类中包含的变量
* 局部变量
* 成员变量
* 类变量（必须声明为static）

## 5、源文件声明规则
* 一个源文件中只能有一个public类；
* 一个源文件可以有多个非public类；
* 源文件的名称应该和public类的类名保持一致；
* 如果一个类定义在某个包中，那么package语句应该在源文件的首行；
* 如果源文件中包含import语句，那么应该放在package语句和类定义之间。如果没有package语句，那么import语句应该在源文件中最前面；
* import语句和package语句对源文件中定义的所有类都有效，在同一源文件中，不能给不同的类不同的包声明。

## 6、java数据类型
* byte（有符号8位整数）
* short
* int
* long
* float
* double
* char

## 7、java常量

```
final double PI = 3.1415927
```

## 8、局部变量和实例变量区别
|局部变量|实例变量|
|:-------------:|:-------------:|
|访问修饰符不能用于局部变量|可用于实例变量|
|局部变量无默认值|有默认值|

## 9、类变量（静态变量）
类变量被声明为public static final类型时，类变量名称必须使用大写字母。

## 10、访问控制
|修饰符|当前类|同一包内|子孙类|其他包|
|:-------------:|:-------------:|:-------------:|:-------------:|:-------------:|
|pulic|√|√|√|√|
|protected|√|√|√|×|
|default|√|√|×|×|
|private|√|×|×|×|
protected访问修饰符不能修饰类和接口，接口中的成员变量和成员方法不能声明为protected。

## 11、访问控制和继承
|父类|子类|
|:-------------:|:-------------:|
|public|必须public|
|protected|protected或public，不能为private|
|private|不能被继承|

## 12、非访问修饰符
* static->类变量和类方法
* final->修饰类不能被继承，方法不能被重写，修饰变量不可修改（接近const）
* Abstract->创建抽象类和抽象方法

## 13、static修饰符
* 静态变量
* 静态方法

## 14、Final修饰符
Final变量能被显式地初始化，并且只能被初始化一次。被声明为final的对象的引用不能指向不同的对象，但是final对象里的数据可以被改变

## 15、Abstract修饰符
* 抽象类不能用来实例化对象，声明抽象类的唯一目的是为了将来对该类进行扩充
* 一个类不能同时被Abstract和final修饰。如果一个类包含抽象方法，那么该类一定要声明为抽象类，否则将出现编译错误
* 抽象类可以包含抽象方法和非抽象方法
* 抽象方法是种没有任何实现的方法，该方法的具体实现由子类提供，抽象方法不能声明为final和static
* 任何继承抽象类的子类必须要实现父类的所有抽象方法，除非子类也是抽象类

## 16、instanceof运算符
用于操作对象实例，检查该对象是否是一个特定类型（类类型或接口类型）

```
String name = "James";
boolean result = name instanceof String; // result返回true
```
如果被比较的对象兼容于右侧类型，改运算符仍然返回true

## 17、for语句

```
for (声明语句:表达式) {
	// 代码语句
}
```

## 18、Number方法

## 19、Java character类

## 20、String类
String（不可改变），如需改变应选择使用StringBuffer和StringBuilder类
* StringBuffer（线程安全）
* StringBuilder（速度快）

## 21、Java休眠（sleep）

```
Thread.sleep(1000*3); // 休眠3秒
```

## 22、构造方法
构造方法和它所在类的名字相同，但构造方法没有返回值

## 23、可变参数
一个方法中只能指定一个可变参数，它必须是方法的最后一个参数，任何普通的参数必须在它之前声明

## 24、finalize()方法
finalize()方法里，你必须指定在对象销毁时要执行的操作

```
protect void finalize() throws java.lang.Throwable {
	super.finalize()
	// 对象销毁时要执行的操作
}
```

## 25、抽象类和接口的区别
* 想让一些方法有默认实现，用抽象类
* 想实现多重继承，用接口

## 26、重写规则
访问权限必须比父类权限高

## 27、重载与重写的区别
|区别点|重载方法|重写方法|
|:-------------:|:-------------:|:-------------:|
|参数列表|必须修改|一定不能修改|
|返回类型|可以修改|一定不能修改|
|异常|可以修改|可以减少式删除|
|访问|可以修改|一定不能做严格的限制（可以降低限制）|

## 28、抽象类、抽象方法

## 29、接口（interface）
接口是抽象方法的集合。一个类实现一个或者多个接口，因此继承了接口的抽象方法。接口的特点：
* 不能实例化
* 没有构造函数
* 所有方法都是抽象的，同时也是隐式的pulic、static
* 只能含有声明为final、static的field

## 30、接口和抽象类的区别
* 抽象类可以有构造方法，接口不行
* 抽象类可以有普通成员变量，接口没有
* 抽象类可以有非抽象方法，接口必须全部抽象
* 抽象类的访问类型都可以，接口只能是pulic abstract
* 一个类可以实现多个接口，但只能继承一个抽象类

## 31、super
注意super的使用，super并没有代表超类的一个引用能力，只是代表调用父类的方法而已

## 32、@Component, @Service, @Controller, @Repository注解的区别
* @Component, @Service, @Controller, @Repository是spring注解，注解后可以被spring框架所扫描并注入到spring容器来进行管理
* @Component是通用注解，其他三个注解是这个注解的拓展，并且具有了特定的功能
* @Repository注解在持久层中，具有将数据库操作抛出的原生异常翻译转化为spring的持久层异常的功能。
* @Controller层是spring-mvc的注解，具有将请求进行转发，重定向的功能。
* @Service层是业务逻辑层注解，这个注解只是标注该类处于业务逻辑层。
* 用这些注解对应用进行分层之后，就能将请求处理，义务逻辑处理，数据库操作处理分离出来，为代码解耦，也方便了以后项目的维护和开发。

## 33、java只使用try和finally不使用catch的原因和场景
JDK并发工具包中，很多异常处理都使用了如下的结构，如AbstractExecutorService，即只有try和finally没有catch。

```
class X   
{  
    private final ReentrantLock lock = new ReentrantLock();  
    // ...  
   
    public void m()  
    {  
    lock.lock();  // block until condition holds  
    try   
    {  
        // ... method body  
    } finally  
    {  
        lock.unlock()  
    }  
     }  
} 
```

为什么要使用这种结构？有什么好处呢？先看下面的代码

```
public void testTryAndFinally(String name)  
{  
       try  
       {  
           name.length();// NullPointerException  
       }  
       finally  
       {  
           System.out.println("aa");  
       }  
} 
```

* 传递null调用该方法的执行结果是：在控制台打印aa，并抛出NullPointerException。即程序的执行流程是先执行try块，出现异常后执行finally块，最后向调用者抛出try中的异常。这种执行结果是很正常的，因为没有catch异常处理器，所有该方法只能将产生的异常向外抛；因为有finally，所以会在方法返回抛出异常之前，先执行finally代码块中的清理工作。
* 这种做法的好处是什么呢？对于testTryAndFinally来说，它做了自己必须要做的事(finally)，并向外抛出自己无法处理的异常；对于调用者来说，能够感知出现的异常，并可以按照需要进行处理。也就是说这种结构实现了职责的分离，实现了异常处理(throw)与异常清理(finally)的解耦，让不同的方法专注于自己应该做的事。
* 那什么时候使用try-finally，什么时候使用try-catch-finally呢？很显然这取决于方法本身是否能够处理try中出现的异常。如果自己可以处理，那么直接catch住，不用抛给方法的调用者；如果自己不知道怎么处理，就应该将异常向外抛，能够让调用者知道发生了异常。即在方法的签名中声明throws可能出现而自己又无法处理的异常，但是在方法内部做自己应该的事情。

java 的异常处理中，
* 在不抛出异常的情况下，程序执行完 try 里面的代码块之后，该方法并不会立即结束，而是继续试图去寻找该方法有没有 finally 的代码块;
* 如果没有finally代码块，整个方法在执行完try代码块后返回相应的值来结束整个方法;
* 如果有finally代码块，此时程序执行到try代码块里的return语句之时并不会立即执行return，而是先去执行finally代码块里的代码;
* 若finally代码块里没有return或没有能够终止程序的代码，程序将在执行完finally代码块代码之后再返回try代码块执行return语句来结束整个方法;
* 若finally代码块里有return或含有能够终止程序的代码，方法将在执行完finally之后被结束，不再跳回try代码块执行return;
* 在抛出异常的情况下，原理也是和上面的一样的，你把上面说到的try换成catch去理解就OK了 *_*

## 34、spring4.x定时任务执行

```
package com.yzhotel.task;

import org.springframework.scheduling.annotation.EnableScheduling;
import org.springframework.scheduling.annotation.Scheduled;
import org.springframework.stereotype.Component;

@Component
@EnableScheduling
public class WxTask {

    @Scheduled(cron = "0/5 * * * * ?")
    public void sendMessage(){
    	System.out.println("定时任务执行了！");
    }
}
```

## 35、Mac环境中Jenkins的停止和启动命令
启动  
sudo launchctl load /Library/LaunchDaemons/org.jenkins-ci.plist  
停止  
sudo launchctl unload /Library/LaunchDaemons/org.jenkins-ci.plist

## 36、web.xml 配置中classpath: 与classpath*:的区别
classpath：只会到你的class路径中查找找文件;   
classpath\*：不仅包含class路径，还包括jar文件中(class路径)进行查找.

## 37、dubbo类型转换
服务提供端：
```
public String getOrganizationList() {
	List<MsOrganization> datas = mapper.getOrganizationList(); 
	String result = (ResponseResult.succ(datas)).toString();
        return result;
}
```
服务消费端：
```
public ResponseResult test() {
	DubboOrganizationService dubboOrganizationService = SpringApplicationContext.context.getBean(DubboOrganizationService.class);
        String str = dubboOrganizationService.getOrganizationList();
        JSONObject jsStr = JSONObject.parseObject(str);
        String statusCode = jsStr.getString("statusCode");
        
        ResponseResult responseResult = (ResponseResult)JSONObject.toJavaObject(jsStr,ResponseResult.class);
        return responseResult;
}
```

## 38、Mybatis使用
### （1）拼接字段
concat(tp.project_name,'-', taf.form_name) project_form_name
### （2）常用Mapper
```
<resultMap type="com.tfkj.ticket.entity.dto.TkArtFormDTO" id="projectResultMap">
	<id property="id" column="id"/>
	<result property="eventToken" column="event_token"/>
	<result property="formDescription" column="form_description"/>
	<result property="formName" column="form_name"/>
	<result property="isChose" column="is_chose"/>
	<result property="restrictionType" column="restriction_type"/>
	<result property="showDate" column="show_date"/>
	<result property="projectId" column="project_id"/>
	<result property="venueVersionId" column="venue_version_id"/>
	<result property="createTime" column="create_time"/>
	<result property="createUser" column="create_user"/>
	<result property="modifyTime" column="modify_time"/>
	<result property="modifyUser" column="modify_user"/>
	<result property="projectName" column="project_name"/>
	<result property="projectFormName" column="project_form_name"/>
	<result property="venueName" column="venue_name"/>
	<result property="isTop" column="is_top"/>
	<result property="topTime" column="top_time"/>
	<result property="formHostStatus" column="form_host_status"/>
</resultMap>
```
```
<select id="getNonPageArtFormList" resultMap="projectResultMap">
		select taf.*, tp.project_name, v.venue_name, concat(tp.project_name,'-', taf.form_name) project_form_name
		from tk_art_form taf
		left join tk_project tp on taf.project_id = tp.id
		left join (select tvv.*, ttv.venue_name from tk_venue_version tvv
			left join tk_the_venue ttv on tvv.venue_id = ttv.id) v
		on taf.venue_version_id = v.id
        WHERE taf.form_delete_status='1113002'
        <if test="formName != null and formName != ''">
            AND LOCATE(#{formName},taf.form_name)&gt;0
        </if>
        <if test="projectId != null and projectId != ''">
            AND taf.project_id=#{projectId}
        </if>
        <if test="showBeginTime != null ">
			AND #{showBeginTime}&lt;taf.show_date
		</if>
		<if test="showEndTime != null ">
			AND #{showEndTime}&gt;taf.show_date
		</if>
		 <if test="formCancelStatus != null and formCancelStatus != ''">
            AND taf.form_cancel_status =#{formCancelStatus}
        </if>
	</select>
```
## 39、@Autowired(required=false)和@Autowired(required=true)
@Autowired(required=true)：当使用@Autowired注解的时候，其实默认就是@Autowired(required=true)，表示注入的时候，该bean必须存在，否则就会注入失败。  
@Autowired(required=false)：表示忽略当前要注入的bean，如果有直接注入，没有跳过，不会报错。
## 40、Maven本地包安装
```
mvn install:install-file  -Dfile=/Users/chenhu/Desktop/taobao.sdk.java-0.0.1.RELEASE.jar  -DgroupId=com.tfkj  -DartifactId=taobao.sdk.java -Dversion=0.0.1.RELEASE -Dpackaging=jar
```
## 41、修改jar包中的配置文件
### 1、通过vim命令直接编辑jar
vim xxx.jar 该命令首先会列出全部文件，可以通过输入/abc来搜索，定位到对应的abc文件后回车进入配置文件内进行编辑，:wq保存。
### 2、通过jar命令替换jar包中的文件(也可新增)
* 列出jar包中的文件清单  
  jar tf genesys_data_etl-0.0.1-SNAPSHOT.jar

* 提取出内部jar包的指定文件  
  jar xf genesys_data_etl-0.0.1-SNAPSHOT.jar BOOT-INF/classes/realtime/t_ivr_data_bj.json

* 然后可以修改文件  
  vim BOOT-INF/classes/realtime/t_ivr_data_bj.json

* 更新配置文件到内部jar包.(存在覆盖，不存在就新增)  
  jar uf genesys_data_etl-0.0.1-SNAPSHOT.jar BOOT-INF/classes/realtime/t_ivr_data_bj.json

* 更新内部jar包到jar文件  
  jar uf genesys_data_etl-0.0.1-SNAPSHOT.jar 内部jar包.jar

* 可以查看验证是否已经更改  
  vim genesys_data_etl-0.0.1-SNAPSHOT.jar

### 3、解压jar包，修改后重新打包jar
* 解压  
  unzip genesys_data_etl-0.0.1-SNAPSHOT.jar
* 移除jar包,最好备份  
  rm -rf genesys_data_etl-0.0.1-SNAPSHOT.jar
* 重新打包  
  jar -cfM0 new-genesys_data_etl-0.0.1-SNAPSHOT.jar *

## 42、Bean与String相互转换
```
/**
     * 将任意类型转换成字符串
     * @param value
     * @param <T>
     * @return
     */
    public static <T> String beanToString(T value) {
        Class<?> clazz = value.getClass();
        if(clazz == int.class || clazz == Integer.class) {
            return value + "";
        }else if(clazz == String.class) {
            return (String)value;
        }else if(clazz == long.class || clazz == Long.class) {
            return value + "";
        }else {
            return JSON.toJSONString(value);
        }

```
```
   /**
     * 把一个字符串转换成bean对象
     * @param str
     * @param <T>
     * @return
     */
    public static <T> T stringToBean(String str, Class<T> clazz) {
        if(str == null || str.length() <= 0 || clazz == null) {
            return null;
        }
        if(clazz == int.class || clazz == Integer.class) {
            return (T)Integer.valueOf(str);
        }else if(clazz == String.class) {
            return (T)str;
        }else if(clazz == long.class || clazz == Long.class) {
            return  (T)Long.valueOf(str);
        }else {
            return JSON.toJavaObject(JSON.parseObject(str), clazz);
        }
```

## Java8 Stream
```
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

public class StreamExample {
    public static void main(String[] args) {
        List<Integer> numbers = Arrays.asList(5, 2, 8, 1, 7, 3, 4, 6);

        // 使用 Stream 进行转换、过滤和排序操作
        List<Integer> result = numbers.stream()
                .map(x -> x * 2) // 将每个元素乘以2
                .filter(x -> x > 5) // 过滤掉小于等于5的元素
                .sorted() // 对元素进行自然排序
                .collect(Collectors.toList()); // 将结果收集到列表中

        // 打印结果
        System.out.println(result);
    }
}
```

## Optional使用
```
import java.util.Optional;

public class OptionalExample {
    public static void main(String[] args) {
        String input = "Hello, World!";

        // 使用 Optional 处理可能为空的字符串
        String result = Optional.ofNullable(input)
                .map(String::toUpperCase) // 如果不为空，将字符串转换为大写
                .orElse("Default Value"); // 如果为空，提供默认值 "Default Value"

        // 打印结果
        System.out.println(result);
    }
}
```

## 综合样例
让我们重新构建一个例子，使用更具体的类和注解，以便更清晰地理解：

假设有一个类 `Task`，它带有 `TaskJob` 注解：

```java
import java.lang.annotation.*;

@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.TYPE)
@interface TaskJob {
    String type();
}

@TaskJob(type = "A")
class TaskA {
    // TaskA 的具体实现
}

@TaskJob(type = "B")
class TaskB {
    // TaskB 的具体实现
}
```

然后，有一个包含这些任务的 Map：

```java
import java.util.*;
import java.util.stream.Collectors;

public class TaskProcessor {
    private Map<String, Object> tasks;

    public TaskProcessor(Map<String, Object> tasks) {
        this.tasks = tasks;
    }

    public List<TaskJobWrapper> processTasks(String projectId, Set<String> typeCache) {
        return tasks.values().stream()
                .map(Object::getClass)
                .filter(c -> c.isAnnotationPresent(TaskJob.class) && typeCache.contains(c.getAnnotation(TaskJob.class).type()))
                .collect(Collectors.toMap(c -> c.getAnnotation(TaskJob.class).type(), c -> c))
                .entrySet().stream()
                .map(entry -> {
                    TaskJobWrapper wrapper = new TaskJobWrapper();
                    wrapper.setType(entry.getKey());
                    wrapper.setProjectId(projectId);
                    wrapper.init(Optional.ofNullable(entry.getValue()));
                    return wrapper;
                }).collect(Collectors.toList());
    }

    public static void main(String[] args) {
        Map<String, Object> taskMap = new HashMap<>();
        taskMap.put("taskA", new TaskA());
        taskMap.put("taskB", new TaskB());

        TaskProcessor taskProcessor = new TaskProcessor(taskMap);
        Set<String> typeCache = new HashSet<>(Arrays.asList("A", "B"));

        List<TaskJobWrapper> wrappers = taskProcessor.processTasks("123", typeCache);

        wrappers.forEach(wrapper -> System.out.println("Type: " + wrapper.getType() + ", ProjectId: " + wrapper.getProjectId()));
    }
}
```

在这个例子中，`TaskJobWrapper` 是一个包装器类，用于封装任务的相关信息。`TaskProcessor` 类的 `processTasks` 方法从任务 Map 中选择出符合条件的任务，并将它们转换为 `TaskJobWrapper` 对象，最后生成一个包含这些对象的列表。在 `main` 方法中，我们创建了一个包含 `TaskA` 和 `TaskB` 的任务 Map，并通过 `TaskProcessor` 处理并打印结果。

## Map<String, Object> beans = SpringUtil.getApplicationContext().getBeansWithAnnotation(TaskJob.class);
这行代码使用 Spring 框架的 `getBeansWithAnnotation` 方法，通过反射获取在 Spring 容器中标注有 `TaskJob` 注解的所有 Bean。让我们逐步解释这行代码：

```java
Map<String, Object> beans = SpringUtil.getApplicationContext().getBeansWithAnnotation(TaskJob.class);
```

1. **`SpringUtil.getApplicationContext()`：** 这部分调用是使用 `SpringUtil` 类中的静态方法 `getApplicationContext()` 来获取 Spring 应用程序上下文（`ApplicationContext`）。`getApplicationContext()` 方法可能是一个自定义的工具方法，其目的是获取当前运行的 Spring 应用程序上下文。

2. **`.getBeansWithAnnotation(TaskJob.class)`：** 在获取到 Spring 应用程序上下文后，调用 `getBeansWithAnnotation` 方法。这个方法是 Spring 框架提供的，用于获取所有标注有指定注解的 Bean。

    - `TaskJob.class` 是一个注解类，`getBeansWithAnnotation(TaskJob.class)` 的调用表示要获取所有标注有 `TaskJob` 注解的 Bean。

    - 返回的结果是一个 `Map`，其中键是 Bean 的名称（在 Spring 容器中注册的名称），值是对应的 Bean 实例。

综合起来，这一行代码的作用是从 Spring 容器中获取所有标注有 `TaskJob` 注解的 Bean，并将它们以键值对的形式存储在一个 `Map<String, Object>` 中，其中键是 Bean 的名称，值是对应的 Bean 实例。这样的操作通常用于在运行时动态地获取和处理具有特定注解的 Bean。
## 注解操作
让我们重新构建一个例子，更加简化并突显核心概念：

假设有一个注解 `CustomAnnotation`：

```java
import java.lang.annotation.*;

@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.TYPE)
@interface CustomAnnotation {
    String value() default "default";
}
```

然后有一个包含了被标注为 `CustomAnnotation` 的类的 Map：

```java
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

public class AnnotationProcessor {

    public void processClassAnnotations(Optional<? extends Class<?>> clazzOptional) {
        // 1. 获取注解信息和初始化一些属性

        // 1.1 获取注解信息
        String annotationValue = clazzOptional.map(c -> c.getAnnotation(CustomAnnotation.class)).map(CustomAnnotation::value).orElseGet(() -> clazzOptional.map(Class::getName).orElse("default"));

        // 1.2 输出结果
        System.out.println("Annotation Value: " + annotationValue);
    }

    public static void main(String[] args) {
        Map<String, Object> classMap = new HashMap<>();
        classMap.put("classA", new ClassA());
        classMap.put("classB", new ClassB());

        AnnotationProcessor processor = new AnnotationProcessor();

        // 对包含了被标注为 CustomAnnotation 的类的 Map 进行处理
        classMap.values().forEach(clazz -> processor.processClassAnnotations(Optional.ofNullable(clazz.getClass())));
    }
}

@CustomAnnotation(value = "ClassAAnnotation")
class ClassA {
    // ClassA 的具体实现
}

@CustomAnnotation(value = "ClassBAnnotation")
class ClassB {
    // ClassB 的具体实现
}
```

在这个例子中：

- 使用 `CustomAnnotation` 注解标注了两个类 `ClassA` 和 `ClassB`。
- `AnnotationProcessor` 类的 `processClassAnnotations` 方法接受一个 `Optional<? extends Class<?>>` 参数，该参数表示一个类或其子类。
- 在方法内部，通过 `clazzOptional.map(c -> c.getAnnotation(CustomAnnotation.class)).map(CustomAnnotation::value)` 获取类上的注解值，如果注解不存在，则使用默认值。
- 在 `main` 方法中，创建了一个包含了被标注为 `CustomAnnotation` 的类的 Map，并对其进行处理。

这个例子简化了代码，突显了通过 `Optional` 处理注解信息和提供默认值的过程。
