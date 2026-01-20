#[derive(std::fmt::Debug, std::clone::Clone, serde::Serialize, serde::Deserialize)]
pub struct FooRequest;
#[derive(std::fmt::Debug, std::clone::Clone, serde::Serialize, serde::Deserialize)]
pub struct FooResponse;

#[derive(std::fmt::Debug, std::clone::Clone, serde::Serialize, serde::Deserialize)]
pub struct BarRequest;
#[derive(std::fmt::Debug, std::clone::Clone, serde::Serialize, serde::Deserialize)]
pub struct BarResponse;

#[dora_schema_macro::dora_schema(encoding = PostcardEncoding)]
pub trait MyProtocol {
    #[allow(clippy::disallowed_names)]
    fn foo(foo: FooRequest) -> FooResponse;
    fn bar(bar: BarRequest) -> BarResponse;
}

#[allow(unused)]
async fn compile_check<H: MyProtocol>(handler: H) {
    let request: MyProtocolRequest = MyProtocolRequest::Foo { foo: FooRequest };
    let _: MyProtocolResponse = handler.handle(request).await.unwrap();
}
