| 创建型 (Creational) | 结构型 (Structural) | 行为型 (Behavioral)               |
|----------------------|---------------------|-----------------------------------|
| 工厂方法 Factory Method | 适配器 Adapter        | 责任链 Chain of Responsibility    |
| 抽象工厂 Abstract Factory | 桥接 Bridge          | 命令 Command                      |
| 建造者 Builder         | 组合 Composite        | 解释器 Interpreter                |
| 原型 Prototype         | 装饰 Decorator        | 迭代器 Iterator                   |
| 单例 Singleton         | 外观 Facade           | 中介 Mediator                     |
|                      | 享元 Flyweight       | 备忘录 Memento                    |
|                      | 代理 Proxy           | 观察者 Observer                   |
|                      |                     | 状态 State                        |
|                      |                     | 策略 Strategy                     |
|                      |                     | 模板方法 Template Method          |
|                      |                     | 访问者 Visitor                    |

* 创建型包含5种模式，涉及对象/对象组合的创建构建。
* 结构性包含7种模式，涉及对象/类之间的关系。
* 行为型包含11种模式，涉及对象/类的行为、状态、流程。

在C++中实现创建型设计模式，最常用的是工厂模式、抽象工厂模式、单例模式、建造者模式和原型模式。下面是每种模式的简单实现例子：

### 1. 工厂模式（Factory Pattern）

工厂模式通过定义一个工厂类来创建对象，而不是在客户端代码中直接实例化对象。

```cpp
#include <iostream>
#include <memory>

class Product {
public:
    virtual void use() = 0;
    virtual ~Product() = default;
};

class ConcreteProductA : public Product {
public:
    void use() override { std::cout << "Using ConcreteProductA\n"; }
};

class ConcreteProductB : public Product {
public:
    void use() override { std::cout << "Using ConcreteProductB\n"; }
};

class Factory {
public:
    std::unique_ptr<Product> createProduct(const std::string& type) {
        if (type == "A")
            return std::make_unique<ConcreteProductA>();
        else if (type == "B")
            return std::make_unique<ConcreteProductB>();
        return nullptr;
    }
};

int main() {
    Factory factory;
    auto productA = factory.createProduct("A");
    auto productB = factory.createProduct("B");
    productA->use();
    productB->use();
    return 0;
}
```

### 2. 抽象工厂模式（Abstract Factory Pattern）

抽象工厂模式提供了一个接口，用于创建一系列相关或依赖的对象。

```cpp
#include <iostream>
#include <memory>

class ProductA {
public:
    virtual void use() = 0;
    virtual ~ProductA() = default;
};

class ProductB {
public:
    virtual void use() = 0;
    virtual ~ProductB() = default;
};

class ConcreteProductA1 : public ProductA {
public:
    void use() override { std::cout << "Using ConcreteProductA1\n"; }
};

class ConcreteProductB1 : public ProductB {
public:
    void use() override { std::cout << "Using ConcreteProductB1\n"; }
};

class ConcreteProductA2 : public ProductA {
public:
    void use() override { std::cout << "Using ConcreteProductA2\n"; }
};

class ConcreteProductB2 : public ProductB {
public:
    void use() override { std::cout << "Using ConcreteProductB2\n"; }
};

class AbstractFactory {
public:
    virtual std::unique_ptr<ProductA> createProductA() = 0;
    virtual std::unique_ptr<ProductB> createProductB() = 0;
    virtual ~AbstractFactory() = default;
};

class ConcreteFactory1 : public AbstractFactory {
public:
    std::unique_ptr<ProductA> createProductA() override {
        return std::make_unique<ConcreteProductA1>();
    }
    std::unique_ptr<ProductB> createProductB() override {
        return std::make_unique<ConcreteProductB1>();
    }
};

class ConcreteFactory2 : public AbstractFactory {
public:
    std::unique_ptr<ProductA> createProductA() override {
        return std::make_unique<ConcreteProductA2>();
    }
    std::unique_ptr<ProductB> createProductB() override {
        return std::make_unique<ConcreteProductB2>();
    }
};

int main() {
    std::unique_ptr<AbstractFactory> factory = std::make_unique<ConcreteFactory1>();
    auto productA = factory->createProductA();
    auto productB = factory->createProductB();
    productA->use();
    productB->use();

    factory = std::make_unique<ConcreteFactory2>();
    productA = factory->createProductA();
    productB = factory->createProductB();
    productA->use();
    productB->use();

    return 0;
}
```

### 3. 单例模式（Singleton Pattern）

单例模式确保类只有一个实例，并提供一个全局访问点。

```cpp
#include <iostream>
#include <memory>

class Singleton {
public:
    static Singleton& getInstance() {
        static Singleton instance;
        return instance;
    }

    void doSomething() {
        std::cout << "Doing something in Singleton\n";
    }

private:
    Singleton() = default;
    Singleton(const Singleton&) = delete;
    Singleton& operator=(const Singleton&) = delete;
};

int main() {
    Singleton::getInstance().doSomething();
    return 0;
}
```

### 4. 建造者模式（Builder Pattern）

建造者模式将一个复杂对象的构建过程与其表示分离。

```cpp
#include <iostream>
#include <memory>

class Product {
public:
    void addPart(const std::string& part) {
        parts += part + " ";
    }

    void show() {
        std::cout << "Product parts: " << parts << '\n';
    }

private:
    std::string parts;
};

class Builder {
public:
    virtual void buildPartA() = 0;
    virtual void buildPartB() = 0;
    virtual std::unique_ptr<Product> getResult() = 0;
    virtual ~Builder() = default;
};

class ConcreteBuilder : public Builder {
public:
    ConcreteBuilder() { product = std::make_unique<Product>(); }

    void buildPartA() override { product->addPart("PartA"); }
    void buildPartB() override { product->addPart("PartB"); }

    std::unique_ptr<Product> getResult() override { return std::move(product); }

private:
    std::unique_ptr<Product> product;
};

class Director {
public:
    void setBuilder(Builder* builder) { this->builder = builder; }

    void construct() {
        builder->buildPartA();
        builder->buildPartB();
    }

private:
    Builder* builder = nullptr;
};

int main() {
    Director director;
    ConcreteBuilder builder;
    director.setBuilder(&builder);

    director.construct();
    auto product = builder.getResult();
    product->show();

    return 0;
}
```

### 5. 原型模式（Prototype Pattern）

原型模式用于创建一个对象的副本。

```cpp
#include <iostream>
#include <memory>

class Prototype {
public:
    virtual std::unique_ptr<Prototype> clone() const = 0;
    virtual void use() = 0;
    virtual ~Prototype() = default;
};

class ConcretePrototype : public Prototype {
public:
    ConcretePrototype(int data) : data(data) {}

    std::unique_ptr<Prototype> clone() const override {
        return std::make_unique<ConcretePrototype>(*this);
    }

    void use() override {
        std::cout << "Using ConcretePrototype with data: " << data << '\n';
    }

private:
    int data;
};

int main() {
    auto prototype = std::make_unique<ConcretePrototype>(42);
    auto clone = prototype->clone();
    
    prototype->use();
    clone->use();

    return 0;
}
```

这些代码展示了创建型设计模式的基本实现。每种模式在不同的情境下应用，可以帮助代码更具扩展性和可维护性。

结构型设计模式主要用于处理类和对象之间的关系，帮助将不同的组件组合成复杂的结构。常见的结构型模式包括：**适配器模式**、**桥接模式**、**组合模式**、**装饰模式**、**外观模式**、**享元模式**和**代理模式**。下面是这些模式的简单实现例子：

### 1. 适配器模式（Adapter Pattern）

适配器模式用于将一个类的接口转换为客户端期望的接口，主要用于让接口不兼容的类一起工作。

```cpp
#include <iostream>
#include <memory>

// 需要适配的接口
class Target {
public:
    virtual void request() = 0;
    virtual ~Target() = default;
};

// 已存在的类（被适配者）
class Adaptee {
public:
    void specificRequest() {
        std::cout << "Specific request in Adaptee\n";
    }
};

// 适配器
class Adapter : public Target {
public:
    Adapter(std::shared_ptr<Adaptee> adaptee) : adaptee(adaptee) {}

    void request() override {
        adaptee->specificRequest();
    }

private:
    std::shared_ptr<Adaptee> adaptee;
};

int main() {
    std::shared_ptr<Adaptee> adaptee = std::make_shared<Adaptee>();
    std::unique_ptr<Target> adapter = std::make_unique<Adapter>(adaptee);
    adapter->request();

    return 0;
}
```

### 2. 桥接模式（Bridge Pattern）

桥接模式通过将实现部分和抽象部分分离，使它们可以独立变化。

```cpp
#include <iostream>
#include <memory>

class Implementor {
public:
    virtual void operationImpl() = 0;
    virtual ~Implementor() = default;
};

class ConcreteImplementorA : public Implementor {
public:
    void operationImpl() override {
        std::cout << "ConcreteImplementorA operation\n";
    }
};

class ConcreteImplementorB : public Implementor {
public:
    void operationImpl() override {
        std::cout << "ConcreteImplementorB operation\n";
    }
};

class Abstraction {
public:
    Abstraction(std::unique_ptr<Implementor> implementor) : implementor(std::move(implementor)) {}

    virtual void operation() {
        implementor->operationImpl();
    }

protected:
    std::unique_ptr<Implementor> implementor;
};

class RefinedAbstraction : public Abstraction {
public:
    RefinedAbstraction(std::unique_ptr<Implementor> implementor) : Abstraction(std::move(implementor)) {}

    void operation() override {
        std::cout << "RefinedAbstraction: ";
        implementor->operationImpl();
    }
};

int main() {
    std::unique_ptr<Abstraction> abstractionA = std::make_unique<RefinedAbstraction>(std::make_unique<ConcreteImplementorA>());
    abstractionA->operation();

    std::unique_ptr<Abstraction> abstractionB = std::make_unique<RefinedAbstraction>(std::make_unique<ConcreteImplementorB>());
    abstractionB->operation();

    return 0;
}
```

### 3. 组合模式（Composite Pattern）

组合模式将对象组合成树形结构以表示“部分-整体”的层次结构。

```cpp
#include <iostream>
#include <vector>
#include <memory>

class Component {
public:
    virtual void operation() = 0;
    virtual void add(std::shared_ptr<Component> component) {}
    virtual void remove(std::shared_ptr<Component> component) {}
    virtual ~Component() = default;
};

class Leaf : public Component {
public:
    void operation() override {
        std::cout << "Leaf operation\n";
    }
};

class Composite : public Component {
public:
    void operation() override {
        std::cout << "Composite operation\n";
        for (auto& child : children) {
            child->operation();
        }
    }

    void add(std::shared_ptr<Component> component) override {
        children.push_back(component);
    }

    void remove(std::shared_ptr<Component> component) override {
        children.erase(std::remove(children.begin(), children.end(), component), children.end());
    }

private:
    std::vector<std::shared_ptr<Component>> children;
};

int main() {
    auto leaf1 = std::make_shared<Leaf>();
    auto leaf2 = std::make_shared<Leaf>();
    auto composite = std::make_shared<Composite>();

    composite->add(leaf1);
    composite->add(leaf2);

    composite->operation();

    return 0;
}
```

### 4. 装饰模式（Decorator Pattern）

装饰模式允许动态地给对象添加行为。

```cpp
#include <iostream>
#include <memory>

class Component {
public:
    virtual void operation() = 0;
    virtual ~Component() = default;
};

class ConcreteComponent : public Component {
public:
    void operation() override {
        std::cout << "ConcreteComponent operation\n";
    }
};

class Decorator : public Component {
public:
    Decorator(std::unique_ptr<Component> component) : component(std::move(component)) {}

    void operation() override {
        component->operation();
    }

protected:
    std::unique_ptr<Component> component;
};

class ConcreteDecorator : public Decorator {
public:
    ConcreteDecorator(std::unique_ptr<Component> component) : Decorator(std::move(component)) {}

    void operation() override {
        Decorator::operation();
        std::cout << "ConcreteDecorator additional operation\n";
    }
};

int main() {
    std::unique_ptr<Component> component = std::make_unique<ConcreteComponent>();
    std::unique_ptr<Component> decoratedComponent = std::make_unique<ConcreteDecorator>(std::move(component));
    decoratedComponent->operation();

    return 0;
}
```

### 5. 外观模式（Facade Pattern）

外观模式为子系统中的一组接口提供一个统一的高层接口。

```cpp
#include <iostream>

class SubsystemA {
public:
    void operationA() {
        std::cout << "SubsystemA operation\n";
    }
};

class SubsystemB {
public:
    void operationB() {
        std::cout << "SubsystemB operation\n";
    }
};

class Facade {
public:
    void operation() {
        subsystemA.operationA();
        subsystemB.operationB();
    }

private:
    SubsystemA subsystemA;
    SubsystemB subsystemB;
};

int main() {
    Facade facade;
    facade.operation();

    return 0;
}
```

### 6. 享元模式（Flyweight Pattern）

享元模式用于减少大量细粒度对象的内存占用。

```cpp
#include <iostream>
#include <unordered_map>
#include <memory>

class Flyweight {
public:
    virtual void operation() = 0;
    virtual ~Flyweight() = default;
};

class ConcreteFlyweight : public Flyweight {
public:
    void operation() override {
        std::cout << "ConcreteFlyweight operation\n";
    }
};

class FlyweightFactory {
public:
    std::shared_ptr<Flyweight> getFlyweight(const std::string& key) {
        if (flyweights.find(key) == flyweights.end()) {
            flyweights[key] = std::make_shared<ConcreteFlyweight>();
        }
        return flyweights[key];
    }

private:
    std::unordered_map<std::string, std::shared_ptr<Flyweight>> flyweights;
};

int main() {
    FlyweightFactory factory;
    auto flyweight = factory.getFlyweight("A");
    flyweight->operation();
    auto flyweight2 = factory.getFlyweight("A"); // Reuses the same instance
    flyweight2->operation();

    return 0;
}
```

### 7. 代理模式（Proxy Pattern）

代理模式用于控制对某个对象的访问，通常用于延迟加载、访问控制或日志记录。

```cpp
#include <iostream>
#include <memory>

class Subject {
public:
    virtual void request() = 0;
    virtual ~Subject() = default;
};

class RealSubject : public Subject {
public:
    void request() override {
        std::cout << "RealSubject request\n";
    }
};

class Proxy : public Subject {
public:
    Proxy(std::unique_ptr<RealSubject> realSubject) : realSubject(std::move(realSubject)) {}

    void request() override {
        std::cout << "Proxy controlling access to RealSubject\n";
        realSubject->request();
    }

private:
    std::unique_ptr<RealSubject> realSubject;
};

int main() {
    std::unique_ptr<RealSubject> realSubject = std::make_unique<RealSubject>();
    Proxy proxy(std::move(realSubject));
    proxy.request();

    return 0;
}
```

这些示例展示了结构型模式的基本实现，用于在不同场景中将类和对象组合在一起。

行为型模式关注对象之间的通信和职责分配。常见的行为型模式包括：**责任链模式**、**命令模式**、**解释器模式**、**迭代器模式**、**中介者模式**、**备忘录模式**、**观察者模式**、**状态模式**、**策略模式**、**模板方法模式**和**访问者模式**。下面是这些模式的简单实现例子：

### 1. 责任链模式（Chain of Responsibility Pattern）

责任链模式让多个对象都有机会处理请求，避免了请求的发送者和接收者之间的耦合。

```cpp
#include <iostream>
#include <memory>

class Handler {
public:
    virtual ~Handler() = default;
    void setNext(std::shared_ptr<Handler> nextHandler) {
        next = nextHandler;
    }

    virtual void handleRequest(int request) {
        if (next) {
            next->handleRequest(request);
        }
    }

protected:
    std::shared_ptr<Handler> next;
};

class ConcreteHandlerA : public Handler {
public:
    void handleRequest(int request) override {
        if (request < 10) {
            std::cout << "ConcreteHandlerA handled request " << request << "\n";
        } else if (next) {
            next->handleRequest(request);
        }
    }
};

class ConcreteHandlerB : public Handler {
public:
    void handleRequest(int request) override {
        if (request >= 10) {
            std::cout << "ConcreteHandlerB handled request " << request << "\n";
        } else if (next) {
            next->handleRequest(request);
        }
    }
};

int main() {
    auto handlerA = std::make_shared<ConcreteHandlerA>();
    auto handlerB = std::make_shared<ConcreteHandlerB>();
    handlerA->setNext(handlerB);

    handlerA->handleRequest(5);
    handlerA->handleRequest(15);

    return 0;
}
```

### 2. 命令模式（Command Pattern）

命令模式将请求封装为对象，使得请求的发送者和接收者解耦。

```cpp
#include <iostream>
#include <memory>
#include <vector>

class Command {
public:
    virtual void execute() = 0;
    virtual ~Command() = default;
};

class Receiver {
public:
    void action() {
        std::cout << "Receiver action\n";
    }
};

class ConcreteCommand : public Command {
public:
    ConcreteCommand(std::shared_ptr<Receiver> receiver) : receiver(receiver) {}

    void execute() override {
        receiver->action();
    }

private:
    std::shared_ptr<Receiver> receiver;
};

class Invoker {
public:
    void setCommand(std::shared_ptr<Command> command) {
        this->command = command;
    }

    void invoke() {
        if (command) {
            command->execute();
        }
    }

private:
    std::shared_ptr<Command> command;
};

int main() {
    auto receiver = std::make_shared<Receiver>();
    auto command = std::make_shared<ConcreteCommand>(receiver);
    Invoker invoker;
    invoker.setCommand(command);
    invoker.invoke();

    return 0;
}
```

### 3. 解释器模式（Interpreter Pattern）

解释器模式用于定义一个解释器来处理语言中的语法和表达式。

```cpp
#include <iostream>
#include <string>
#include <memory>
#include <unordered_map>

class Context {
public:
    void setVariable(const std::string& name, bool value) {
        variables[name] = value;
    }

    bool getVariable(const std::string& name) const {
        return variables.at(name);
    }

private:
    std::unordered_map<std::string, bool> variables;
};

class Expression {
public:
    virtual bool interpret(const Context& context) const = 0;
    virtual ~Expression() = default;
};

class VariableExpression : public Expression {
public:
    VariableExpression(const std::string& name) : name(name) {}

    bool interpret(const Context& context) const override {
        return context.getVariable(name);
    }

private:
    std::string name;
};

class AndExpression : public Expression {
public:
    AndExpression(std::unique_ptr<Expression> left, std::unique_ptr<Expression> right)
        : left(std::move(left)), right(std::move(right)) {}

    bool interpret(const Context& context) const override {
        return left->interpret(context) && right->interpret(context);
    }

private:
    std::unique_ptr<Expression> left;
    std::unique_ptr<Expression> right;
};

int main() {
    Context context;
    context.setVariable("x", true);
    context.setVariable("y", false);

    auto expr = std::make_unique<AndExpression>(
        std::make_unique<VariableExpression>("x"),
        std::make_unique<VariableExpression>("y")
    );

    std::cout << "Result of expression: " << expr->interpret(context) << "\n";

    return 0;
}
```

### 4. 迭代器模式（Iterator Pattern）

迭代器模式提供了一种方法来顺序访问集合中的元素，而不暴露其内部表示。

```cpp
#include <iostream>
#include <vector>
#include <memory>

class Iterator {
public:
    virtual bool hasNext() = 0;
    virtual int next() = 0;
    virtual ~Iterator() = default;
};

class ConcreteIterator : public Iterator {
public:
    ConcreteIterator(const std::vector<int>& data) : data(data) {}

    bool hasNext() override {
        return index < data.size();
    }

    int next() override {
        return data[index++];
    }

private:
    std::vector<int> data;
    size_t index = 0;
};

class Aggregate {
public:
    virtual std::unique_ptr<Iterator> createIterator() = 0;
    virtual ~Aggregate() = default;
};

class ConcreteAggregate : public Aggregate {
public:
    ConcreteAggregate() {
        data = {1, 2, 3, 4, 5};
    }

    std::unique_ptr<Iterator> createIterator() override {
        return std::make_unique<ConcreteIterator>(data);
    }

private:
    std::vector<int> data;
};

int main() {
    ConcreteAggregate aggregate;
    auto iterator = aggregate.createIterator();

    while (iterator->hasNext()) {
        std::cout << iterator->next() << " ";
    }

    return 0;
}
```

### 5. 中介者模式（Mediator Pattern）

中介者模式定义了一个对象来封装对象之间的交互。

```cpp
#include <iostream>
#include <memory>
#include <string>

class Mediator;

class Colleague {
public:
    Colleague(Mediator* mediator) : mediator(mediator) {}
    virtual void send(const std::string& message) = 0;
    virtual void receive(const std::string& message) = 0;
    virtual ~Colleague() = default;

protected:
    Mediator* mediator;
};

class Mediator {
public:
    virtual void send(const std::string& message, Colleague* colleague) = 0;
    virtual ~Mediator() = default;
};

class ConcreteColleagueA : public Colleague {
public:
    using Colleague::Colleague;

    void send(const std::string& message) override {
        mediator->send(message, this);
    }

    void receive(const std::string& message) override {
        std::cout << "ColleagueA received: " << message << "\n";
    }
};

class ConcreteColleagueB : public Colleague {
public:
    using Colleague::Colleague;

    void send(const std::string& message) override {
        mediator->send(message, this);
    }

    void receive(const std::string& message) override {
        std::cout << "ColleagueB received: " << message << "\n";
    }
};

class ConcreteMediator : public Mediator {
public:
    void setColleagues(Colleague* a, Colleague* b) {
        colleagueA = a;
        colleagueB = b;
    }

    void send(const std::string& message, Colleague* colleague) override {
        if (colleague == colleagueA) {
            colleagueB->receive(message);
        } else if (colleague == colleagueB) {
            colleagueA->receive(message);
        }
    }

private:
    Colleague* colleagueA = nullptr;
    Colleague* colleagueB = nullptr;
};

int main() {
    ConcreteMediator mediator;
    ConcreteColleagueA colleagueA(&mediator);
    ConcreteColleagueB colleagueB(&mediator);
    mediator.setColleagues(&colleagueA, &colleagueB);

    colleagueA.send("Hello from A!");
    colleagueB.send("Hello from B!");

    return 0;
}
```

上述示例展示了行为型模式的实现。不同的模式在特定场景下有不同的用处，可以帮助简化对象间的交互和行为控制。

确实，行为型模式还包括其他一些常见的模式。以下是其他几个行为型模式的示例代码：

### 6. 备忘录模式（Memento Pattern）

备忘录模式允许保存和恢复对象的状态，而不暴露对象的内部表示。

```cpp
#include <iostream>
#include <string>
#include <memory>

class Memento {
public:
    Memento(const std::string& state) : state(state) {}
    std::string getState() const { return state; }

private:
    std::string state;
};

class Originator {
public:
    void setState(const std::string& state) {
        this->state = state;
        std::cout << "State set to " << state << "\n";
    }

    std::string getState() const { return state; }

    std::shared_ptr<Memento> saveStateToMemento() const {
        return std::make_shared<Memento>(state);
    }

    void restoreStateFromMemento(const std::shared_ptr<Memento>& memento) {
        state = memento->getState();
        std::cout << "State restored to " << state << "\n";
    }

private:
    std::string state;
};

class Caretaker {
public:
    void addMemento(const std::shared_ptr<Memento>& memento) {
        mementos.push_back(memento);
    }

    std::shared_ptr<Memento> getMemento(int index) {
        if (index < mementos.size()) {
            return mementos[index];
        }
        return nullptr;
    }

private:
    std::vector<std::shared_ptr<Memento>> mementos;
};

int main() {
    Originator originator;
    Caretaker caretaker;

    originator.setState("State1");
    caretaker.addMemento(originator.saveStateToMemento());

    originator.setState("State2");
    caretaker.addMemento(originator.saveStateToMemento());

    originator.setState("State3");
    originator.restoreStateFromMemento(caretaker.getMemento(0));

    return 0;
}
```

### 7. 观察者模式（Observer Pattern）

观察者模式定义了一种一对多的依赖关系，当一个对象的状态发生变化时，所有依赖它的对象都会收到通知并自动更新。

```cpp
#include <iostream>
#include <string>
#include <vector>
#include <memory>

class Observer {
public:
    virtual void update(const std::string& message) = 0;
    virtual ~Observer() = default;
};

class ConcreteObserver : public Observer {
public:
    ConcreteObserver(const std::string& name) : name(name) {}

    void update(const std::string& message) override {
        std::cout << name << " received update: " << message << "\n";
    }

private:
    std::string name;
};

class Subject {
public:
    void addObserver(std::shared_ptr<Observer> observer) {
        observers.push_back(observer);
    }

    void removeObserver(std::shared_ptr<Observer> observer) {
        observers.erase(std::remove(observers.begin(), observers.end(), observer), observers.end());
    }

    void notifyObservers(const std::string& message) {
        for (auto& observer : observers) {
            observer->update(message);
        }
    }

private:
    std::vector<std::shared_ptr<Observer>> observers;
};

int main() {
    auto observer1 = std::make_shared<ConcreteObserver>("Observer1");
    auto observer2 = std::make_shared<ConcreteObserver>("Observer2");

    Subject subject;
    subject.addObserver(observer1);
    subject.addObserver(observer2);

    subject.notifyObservers("New update available!");

    subject.removeObserver(observer1);
    subject.notifyObservers("Another update!");

    return 0;
}
```

### 8. 状态模式（State Pattern）

状态模式允许对象在其内部状态改变时改变其行为。

```cpp
#include <iostream>
#include <memory>

class Context;

class State {
public:
    virtual void handle(Context& context) = 0;
    virtual ~State() = default;
};

class Context {
public:
    Context(std::unique_ptr<State> state) : state(std::move(state)) {}

    void setState(std::unique_ptr<State> newState) {
        state = std::move(newState);
    }

    void request() {
        state->handle(*this);
    }

private:
    std::unique_ptr<State> state;
};

class ConcreteStateA : public State {
public:
    void handle(Context& context) override {
        std::cout << "State A handling request and transitioning to State B\n";
        context.setState(std::make_unique<ConcreteStateB>());
    }
};

class ConcreteStateB : public State {
public:
    void handle(Context& context) override {
        std::cout << "State B handling request and transitioning to State A\n";
        context.setState(std::make_unique<ConcreteStateA>());
    }
};

int main() {
    Context context(std::make_unique<ConcreteStateA>());
    context.request();
    context.request();
    context.request();

    return 0;
}
```

### 9. 策略模式（Strategy Pattern）

策略模式定义了算法族，分别封装起来，让它们可以互相替换。

```cpp
#include <iostream>
#include <memory>

class Strategy {
public:
    virtual void algorithm() = 0;
    virtual ~Strategy() = default;
};

class ConcreteStrategyA : public Strategy {
public:
    void algorithm() override {
        std::cout << "Algorithm A\n";
    }
};

class ConcreteStrategyB : public Strategy {
public:
    void algorithm() override {
        std::cout << "Algorithm B\n";
    }
};

class Context {
public:
    void setStrategy(std::unique_ptr<Strategy> strategy) {
        this->strategy = std::move(strategy);
    }

    void executeStrategy() {
        if (strategy) {
            strategy->algorithm();
        }
    }

private:
    std::unique_ptr<Strategy> strategy;
};

int main() {
    Context context;
    context.setStrategy(std::make_unique<ConcreteStrategyA>());
    context.executeStrategy();

    context.setStrategy(std::make_unique<ConcreteStrategyB>());
    context.executeStrategy();

    return 0;
}
```

### 10. 模板方法模式（Template Method Pattern）

模板方法模式定义一个算法的框架，将一些步骤延迟到子类中。

```cpp
#include <iostream>

class AbstractClass {
public:
    virtual ~AbstractClass() = default;

    void templateMethod() {
        step1();
        step2();
        step3();
    }

protected:
    virtual void step1() {
        std::cout << "Step 1\n";
    }

    virtual void step2() = 0;

    virtual void step3() {
        std::cout << "Step 3\n";
    }
};

class ConcreteClass : public AbstractClass {
protected:
    void step2() override {
        std::cout << "Concrete Step 2\n";
    }
};

int main() {
    ConcreteClass obj;
    obj.templateMethod();

    return 0;
}
```

### 11. 访问者模式（Visitor Pattern）

访问者模式表示一种操作，它可以不改变被访问类的结构下访问其元素。

```cpp
#include <iostream>
#include <vector>
#include <memory>

class ConcreteElementA;
class ConcreteElementB;

class Visitor {
public:
    virtual void visitConcreteElementA(ConcreteElementA* element) = 0;
    virtual void visitConcreteElementB(ConcreteElementB* element) = 0;
    virtual ~Visitor() = default;
};

class Element {
public:
    virtual void accept(Visitor* visitor) = 0;
    virtual ~Element() = default;
};

class ConcreteElementA : public Element {
public:
    void accept(Visitor* visitor) override {
        visitor->visitConcreteElementA(this);
    }
};

class ConcreteElementB : public Element {
public:
    void accept(Visitor* visitor) override {
        visitor->visitConcreteElementB(this);
    }
};

class ConcreteVisitor : public Visitor {
public:
    void visitConcreteElementA(ConcreteElementA* element) override {
        std::cout << "ConcreteVisitor visiting ConcreteElementA\n";
    }

    void visitConcreteElementB(ConcreteElementB* element) override {
        std::cout << "ConcreteVisitor visiting ConcreteElementB\n";
    }
};

int main() {
    std::vector<std::unique_ptr<Element>> elements;
    elements.push_back(std::make_unique<ConcreteElementA>());
    elements.push_back(std::make_unique<ConcreteElementB>());

    ConcreteVisitor visitor;
    for (auto& element : elements) {
        element->accept(&visitor);
    }

    return 0;
}
```

这些例子展示了完整的行为型模式，适用于不同的对象行为管理和交互需求。
