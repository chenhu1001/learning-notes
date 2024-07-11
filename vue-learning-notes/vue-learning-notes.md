# vue-learning-notes

## 1、npm换源
* 查看源
```
npm config get registry

// 或
npm config list
```
* 修改源
```
# 默认源
npm config set registry https://registry.npmjs.org

# https -> http，这样网速就会好很多
npm config set registry http://registry.npmjs.org 

# 如果还不能下载，就切换成淘宝源
npm config set registry https://registry.npm.taobao.org
```
* 如果只是临时改变源，可以这样
```
npm --registry=https://registry.npm.taobao.org
```

## 2、npm常用命令
```
// 查找命令
npm search element-ui --registry=https://registry.npmjs.org

// --save参数表示将该模块写入dependencies属性
npm install express --save

// --save-dev表示将该模块写入devDependencies属性
npm install express --save-dev
```

## 3、Vue的api跨域请求问题

使用vue做跨域请求的时候的基本操作是
```javascript

// config/index.js
module.exports = {
  // ...
  dev: {
    proxyTable: {
      // proxy all requests starting with /api to jsonplaceholder
      '/api/**': {
        target: 'http://jsonplaceholder.typicode.com',
        changeOrigin: true,
        pathRewrite: {
          '^/api' :'/'
        }
      }
    }
  }
}
```
(详情见：[https://vuejs-templates.github.io/webpack/proxy.html](https://vuejs-templates.github.io/webpack/proxy.html))

相当于开发环境下的``'/api'``路由是指向xxx.com地址的,同时重定向了你的路由所有带``'/api'``的都是空，即用axios请求数据的时候控制台里看到的其实并没有``/api``这四个字符。``changeOrigin:true``则会自动给请求头加入Host首部。

**但是这个时候就会有一个问题，即一旦项目中的路由一多起来，难免会出现路由的冲突问题，并且上面这个常规操作只是让浏览器认为它没有``'/api'``这个字符，我们码畜还得自己手码啊！每次多打四个字多费时间啊！于是为了懒，我们就要找稍微高端点的操作了！**

整一个变量，让这个变量代替``'/api'``，那tm不就少打字了吗？于是在入口文件引入axios的时候同时把这个变量定下来：
```javascript

// main.js

import axios from "axios"

axios.defaults.baseURL = 'api'
```
好了，这个时候再请求数据的时候就可以少打四个字了。

同样的，这里还是会存在一个弊端，那就算不能区分到底是生产环境还是开发环境。那么为了区分这个情况，我们在config目录下新建一个api.config.js文件来区分它。

```javascript

const isPro = Object.is(process.env.NODE_ENV, 'production')

module.exports = {
  baseURL: isPro ? 'http://jsonplaceholder.typicode.com/api/' :'api/'
}
```
有了这个文件以后，再在入口文件里用它。
```javascript

// main.js
import Vue from 'vue'
import App from './App'
import router from './router'
import axios from "axios"

import apiConfig from '../config/api.config'
axios.defaults.baseURL = apiConfig.baseURL
```

这样就一劳永逸啦！

## 4、服务器实现跨域REST请求
* 配置web.xml
```
<!-- 通过CORS技术实现AJAX跨域访问 -->
<filter>
    <filter-name>corsFilter</filter-name>
    <filter-class>cn.edu.tju.rico.filter.CorsFilter</filter-class>
    <init-param>
        <param-name>allowOrigin</param-name>
        <param-value>http://localhost:8080</param-value>
    </init-param>
    <init-param>
        <param-name>allowMethods</param-name>
        <param-value>GET,POST,PUT,DELETE,OPTIONS</param-value>
    </init-param>
    <init-param>
        <param-name>allowCredentials</param-name>
        <param-value>true</param-value>
    </init-param>
    <init-param>
        <param-name>allowHeaders</param-name>
        <param-value>Content-Type,X-Token</param-value>
    </init-param>
</filter>
<filter-mapping>
    <filter-name>corsFilter</filter-name>
    <url-pattern>/*</url-pattern>
</filter-mapping>
```
* 添加CorsFilter
```
package cn.edu.tju.rico.filter;

import java.io.IOException;
import java.util.Arrays;
import java.util.List;

import javax.servlet.Filter;
import javax.servlet.FilterChain;
import javax.servlet.FilterConfig;
import javax.servlet.ServletException;
import javax.servlet.ServletRequest;
import javax.servlet.ServletResponse;
import javax.servlet.http.HttpServletRequest;
import javax.servlet.http.HttpServletResponse;

import cn.edu.tju.rico.utils.CollectionUtil;
import cn.edu.tju.rico.utils.StringUtil;

/**        
 * Title: 跨域访问处理(跨域资源共享)    
 * Description: 解决前后端分离架构中的跨域问题
 * @author rico       
 * @created 2017年7月4日 下午5:00:09    
 */      
public class CorsFilter implements Filter {
    private String allowOrigin;
    private String allowMethods;
    private String allowCredentials;
    private String allowHeaders;
    private String exposeHeaders;

    @Override
    public void init(FilterConfig filterConfig) throws ServletException {
        allowOrigin = filterConfig.getInitParameter("allowOrigin");
        allowMethods = filterConfig.getInitParameter("allowMethods");
        allowCredentials = filterConfig.getInitParameter("allowCredentials");
        allowHeaders = filterConfig.getInitParameter("allowHeaders");
        exposeHeaders = filterConfig.getInitParameter("exposeHeaders");
    }


    /** 
     * @description 通过CORS技术实现AJAX跨域访问,只要将CORS响应头写入response对象中即可
     * @author rico       
     * @created 2017年7月4日 下午5:02:38      
     * @param req
     * @param res
     * @param chain
     * @throws IOException
     * @throws ServletException     
     * @see javax.servlet.Filter#doFilter(javax.servlet.ServletRequest, javax.servlet.ServletResponse, javax.servlet.FilterChain)     
     */  
    @Override
    public void doFilter(ServletRequest req, ServletResponse res,
            FilterChain chain) throws IOException, ServletException {
        HttpServletRequest request = (HttpServletRequest) req;
        HttpServletResponse response = (HttpServletResponse) res;
        String currentOrigin = request.getHeader("Origin");
        if (StringUtil.isNotEmpty(allowOrigin)) {
            List<String> allowOriginList = Arrays
                    .asList(allowOrigin.split(","));
            if (CollectionUtil.isNotEmpty(allowOriginList)) {
                if (allowOriginList.contains(currentOrigin)) {
                    response.setHeader("Access-Control-Allow-Origin",
                            currentOrigin);
                }
            }
        }
        if (StringUtil.isNotEmpty(allowMethods)) {
            response.setHeader("Access-Control-Allow-Methods", allowMethods);
        }
        if (StringUtil.isNotEmpty(allowCredentials)) {
            response.setHeader("Access-Control-Allow-Credentials",
                    allowCredentials);
        }
        if (StringUtil.isNotEmpty(allowHeaders)) {
            response.setHeader("Access-Control-Allow-Headers", allowHeaders);
        }
        if (StringUtil.isNotEmpty(exposeHeaders)) {
            response.setHeader("Access-Control-Expose-Headers", exposeHeaders);
        }
        chain.doFilter(req, res);
    }

    @Override
    public void destroy() {
    }
}
```
* 工具类
```
StringUtil

package cn.edu.tju.rico.utils;

  
/**        
 * Title: 字符串工具类    
 * @author rico       
 * @created 2017年7月4日 下午5:15:29    
 */      
public class StringUtil {
	  
	/**     
	 * @description 给定字符串是否为空或空串
	 * @author rico       
	 * @created 2017年7月4日 下午5:15:46     
	 * @param str
	 * @return     
	 */
	public static boolean isNotEmpty(String str) {
		if (str != null && str.length() != 0) {
			return true;
		}
		return false;
	}

	/**     
	 * @description 给定字符串是否为空或空串
	 * @author rico       
	 * @created 2017年7月4日 下午5:15:46     
	 * @param str
	 * @return     
	 */
	public static boolean isEmpty(String str) {
		if (str != null && str.length() != 0) {
			return false;
		}
		return true;
	}
}
```
```
CollectionUtil

package cn.edu.tju.rico.utils;

import java.util.Collection;

  
/**        
 * Title: Collection 工具类    
 * Description: 
 * @author rico       
 * @created 2017年7月4日 下午5:14:01    
 */      
public class CollectionUtil {
	public static boolean isNotEmpty(Collection<?> c){
		if (c != null && c.size() != 0 ) {
			return true;
		}
		return false;
	}
}
```
## 5、watch钩子函数的用法
```
watch: {
  radio: {
    handler: function (val, oldval) {
      console.log(val)
    }
  }
}
```

## 6、使用prop传递数据（静态数据传递）
子组件

```
Vue.component('child', {
    // 声明props
    props: ['message'],
    
    // 就像data一样，prop也可以在模块中使用
    template:'<span>{{message}}</span>'
}
```

父组件

```
<child message="hello!"></child>
```

## 7、传递prop数据时，需做相应转换
子组件

```
props: ['myMessage']
```

父组件

```
<child my-message="hello"></child>
```

## 8、动态prop
父组件

```
<child v-bind:my-message="parentMsg"></child>
```

等价于

```
<child :my-message="parentMsg"></child>
```

动态prop传递（传递对象）

```
todo: {
    text: 'Lean Vue',
    isComplete: false
}
```

```
<todo-item v-bind="todo"></todo-item>
```

等价于

```
<todo-item v-bind:text="todo.text" v-bind:is-complete="todo.isComplete"></to-item>
```

## 9、字面量语法VS动态语法
```
<!--传递了一个字符串"|"-->
<comp some-prop="|"></comp>
```

```
<!--传递真正的数值-->
<comp v-bind:some-prop="|"></comp>
```

## 10、单向数据流
* 定义局部变量

```
props: [''initialCounter]
data: function() {
    return {
        counter: this.initalCounter
    }
}
```

* 定义一个计算属性

```
props: ['size']
computed: {
    normalizedSize: function () {
        return this.size.trim().toLowerCase()
        }
}
```

## 11、prop验证
我们可以为组件的prop指定验证规则，如果传入的数据不符合要求，vue会发出警告。

## 12、非prop特性
可直接把属性添加到组件上（不需要事先定义prop）

```
<bs-date-input data-3d-date-picker="true"></bs-date-input>
```

vue会合并属性，class属性会叠加。

## 13、vue单页面，多路由，前进刷新，后退不刷新
https://segmentfault.com/a/1190000012083511?_ea=2864002

## 14、Mac搭建Vue开发环境
```
// 安装Homebrew
/usr/bin/ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
// 安装node.js 
brew install nodejs
// 获取nodejs模块安装目录访问权限
sudo chmod -R 777 /usr/local/lib/node_modules/
// 安装webpack
npm install webpack -g
// 安装vue脚手架
npm install vue-cli -g
```

## 15、Vue利用百度地图定位
1、在主目录下的index.html引入js
```
<script type="text/javascript" src="http://api.map.baidu.com/api?v=2.0&ak=xxxxxxxxxxxxx"></script>
```
2、在webpack.base.conf.js配置文件中配置BMap，在module.exports 中与entry平级，例如：
```
let webpackConfig = {
  context: path.resolve(__dirname, '../'),
  entry: {
    app: './src/main.js'
  },
  externals: {
    'BMap': 'BMap'
  },
  output: {
    path: config.build.assetsRoot,
    filename: '[name].js',
    publicPath: process.env.NODE_ENV === 'production'
      ? config.build.assetsPublicPath
      : config.dev.assetsPublicPath
  },
  resolve: {
    extensions: ['.js', '.vue', '.json'],
    alias: {
      'vue$': 'vue/dist/vue.esm.js',
      '@': resolve('src'),
    }
  }
```
3、在项目中引入BMap
```
import BMap from 'BMap'
```
4、代码
```
addressDetail () {
        let myCity = new BMap.LocalCity()
        myCity.get((result) => {
          console.log('当前定位城市：' + result.name)
          console.log('当前定位城市code：' + result.code)
        })

        var geolocation = new BMap.Geolocation()
        geolocation.getCurrentPosition((r) => {
          if (r.point) {
            console.log(r.longitude)
            console.log(r.latitude)
            var myGeo = new BMap.Geocoder()
            myGeo.getLocation(new BMap.Point(r.longitude, r.latitude), function (result) {
              if (result) {
                console.log(result.address)
              }
            })
          }
        }, { enableHighAccuracy: true })
      }
```
## 16、flex布局
```
  .content {
    margin-top: 5px;
    width: 10rem;
    display: flex;
    flex-wrap: wrap;
    align-content: flex-start;
    background-color: white;
  }

  .item {
    flex: 0 0 20.0% !important;
    box-sizing: border-box;
  }
```
## 17、根据数据动态生成class
```
<i :class="iconType" @click="handleButtonClick"></i>
computed: {
    iconType () {
      return this.collapsed ? 'iconfont icon-zhankai-you' : 'iconfont icon-shouqi-zuo'
    }
}
```
