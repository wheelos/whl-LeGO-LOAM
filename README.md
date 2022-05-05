# apollo-lego-loam
Porting lego-loam to cyber architecture

## User Guide
Create a component, inherit from `cyber::Component`.
```c++
class YourComponent final : public cyber::Component<> {
 public:

 private:

};

CYBER_REGISTER_COMPONENT(YourComponent)
```
