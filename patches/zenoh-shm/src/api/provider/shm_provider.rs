//
// Copyright (c) 2023 ZettaScale Technology
//
// This program and the accompanying materials are made available under the
// terms of the Eclipse Public License 2.0 which is available at
// http://www.eclipse.org/legal/epl-2.0, or the Apache License, Version 2.0
// which is available at https://www.apache.org/licenses/LICENSE-2.0.
//
// SPDX-License-Identifier: EPL-2.0 OR Apache-2.0
//
// Contributors:
//   ZettaScale Zenoh Team, <zenoh@zettascale.tech>
//

use std::{
    error::Error,
    future::{Future, IntoFuture},
    marker::PhantomData,
    mem::MaybeUninit,
    num::NonZeroUsize,
    pin::Pin,
    sync::{atomic::Ordering, Mutex},
    time::Duration,
};

use async_trait::async_trait;
use zenoh_core::{zlock, zresult::ZResult, Resolvable, Wait};

use super::{
    chunk::{AllocatedChunk, ChunkDescriptor},
    shm_provider_backend::ShmProviderBackend,
    types::{ChunkAllocResult, ZAllocError, ZLayoutAllocError, ZLayoutError},
};
use crate::{
    api::{
        buffer::{typed::Typed, zshmmut::ZShmMut},
        protocol_implementations::posix::posix_shm_provider_backend::{
            PosixShmProviderBackend, PosixShmProviderBackendBuilder,
        },
        provider::memory_layout::{MemoryLayout, TypedLayout},
    },
    metadata::{
        allocated_descriptor::AllocatedMetadataDescriptor, descriptor::MetadataDescriptor,
        storage::GLOBAL_METADATA_STORAGE,
    },
    watchdog::{
        confirmator::{ConfirmedDescriptor, GLOBAL_CONFIRMATOR},
        validator::GLOBAL_VALIDATOR,
    },
    ShmBufInfo, ShmBufInner,
};

/// The type of allocation.
pub trait AllocLayout {
    /// The buffer returned by the allocation.
    type Buffer;
    /// Returns the memory layouts of the allocation.
    fn memory_layout(self) -> Result<MemoryLayout, ZLayoutError>;
    /// Wraps the raw allocated buffer.
    ///
    /// # Safety
    ///
    /// The buffer must have the layout returned by [`Self::memory_layout`]
    unsafe fn wrap_buffer(buffer: ZShmMut) -> Self::Buffer;
}

impl<T: TryInto<MemoryLayout>> AllocLayout for T
where
    T::Error: Into<ZLayoutError>,
{
    type Buffer = ZShmMut;

    fn memory_layout(self) -> Result<MemoryLayout, ZLayoutError> {
        self.try_into().map_err(Into::into)
    }

    unsafe fn wrap_buffer(buffer: ZShmMut) -> Self::Buffer {
        buffer
    }
}

impl<T> AllocLayout for TypedLayout<T> {
    type Buffer = Typed<MaybeUninit<T>, ZShmMut>;

    fn memory_layout(self) -> Result<MemoryLayout, ZLayoutError> {
        Ok(MemoryLayout::for_type::<T>())
    }

    unsafe fn wrap_buffer(buffer: ZShmMut) -> Self::Buffer {
        // SAFETY: same precondition
        unsafe { Typed::new_unchecked(buffer) }
    }
}

/// A layout for allocations.
///
/// This is a pre-calculated layout suitable for making series of similar allocations
/// adopted for particular ShmProvider
#[zenoh_macros::unstable_doc]
#[derive(Debug)]
pub struct PrecomputedLayout<'a, Backend, Layout> {
    size: NonZeroUsize,
    provider_layout: MemoryLayout,
    provider: &'a ShmProvider<Backend>,
    _phantom: PhantomData<Layout>,
}

impl<'a, Backend, Layout> PrecomputedLayout<'a, Backend, Layout>
where
    Backend: ShmProviderBackend,
{
    /// Allocate the new buffer with this layout
    #[zenoh_macros::unstable_doc]
    pub fn alloc<'b>(&'b self) -> PrecomputedAllocBuilder<'b, 'a, Backend, Layout> {
        PrecomputedAllocBuilder {
            layout: self,
            policy: JustAlloc,
        }
    }
}

impl<'a, Backend, Layout> PrecomputedLayout<'a, Backend, Layout>
where
    Backend: ShmProviderBackend,
{
    fn new(
        provider: &'a ShmProvider<Backend>,
        required_layout: MemoryLayout,
    ) -> Result<Self, ZLayoutError> {
        // NOTE: Depending on internal implementation, provider's backend might relayout
        // the allocations for bigger alignment (ex. 4-byte aligned allocation to 8-bytes aligned)

        // calculate required layout size
        let size = required_layout.size();

        // Obtain provider's layout for our layout
        let provider_layout = provider
            .backend
            .layout_for(required_layout)
            .map_err(|_| ZLayoutError::ProviderIncompatibleLayout)?;

        Ok(Self {
            size,
            provider_layout,
            provider,
            _phantom: PhantomData,
        })
    }
}

/// Trait for allocation policies
#[zenoh_macros::unstable_doc]
pub trait AllocPolicy<Backend> {
    fn alloc(&self, layout: &MemoryLayout, provider: &ShmProvider<Backend>) -> ChunkAllocResult;
}

/// Trait for async allocation policies
#[zenoh_macros::unstable_doc]
#[async_trait]
pub trait AsyncAllocPolicy<Backend>: Send {
    async fn alloc_async(
        &self,
        layout: &MemoryLayout,
        provider: &ShmProvider<Backend>,
    ) -> ChunkAllocResult;
}

/// Marker trait for policies that are safe to use
///
/// # Safety
///
/// Policies implementing this trait must be safe to use. For example, [`Deallocate`] is not safe
/// as it may deallocate and reuse a buffer that is currently in use. Users of unsafe policy must
/// enforce the safety of their code by other means.
pub unsafe trait SafePolicy {}

/// Marker trait for compile-time allocation policies
#[zenoh_macros::unstable_doc]
pub trait ConstPolicy {
    const NEW: Self;
}

/// A policy value that can be either constant or runtime provided.
pub trait PolicyValue<T> {
    fn get(&self) -> T;
}

impl<T: Copy> PolicyValue<T> for T {
    fn get(&self) -> T {
        *self
    }
}
/// A const `bool`.
pub struct ConstBool<const VALUE: bool>;
impl<const VALUE: bool> ConstPolicy for ConstBool<VALUE> {
    const NEW: Self = Self;
}
impl<const VALUE: bool> PolicyValue<bool> for ConstBool<VALUE> {
    fn get(&self) -> bool {
        VALUE
    }
}
/// A const `usize`.
pub struct ConstUsize<const VALUE: usize>;
impl<const VALUE: usize> ConstPolicy for ConstUsize<VALUE> {
    const NEW: Self = Self;
}
impl<const VALUE: usize> PolicyValue<usize> for ConstUsize<VALUE> {
    fn get(&self) -> usize {
        VALUE
    }
}

/// Just try to allocate
#[zenoh_macros::unstable_doc]
#[derive(Clone, Copy)]
pub struct JustAlloc;
impl<Backend> AllocPolicy<Backend> for JustAlloc
where
    Backend: ShmProviderBackend,
{
    fn alloc(&self, layout: &MemoryLayout, provider: &ShmProvider<Backend>) -> ChunkAllocResult {
        provider.backend.alloc(layout)
    }
}
unsafe impl SafePolicy for JustAlloc {}
impl ConstPolicy for JustAlloc {
    const NEW: Self = Self;
}

/// Garbage collection policy.
///
/// Try to reclaim old buffers if allocation failed and allocate again
/// if the largest reclaimed chuk is not smaller than the one required.
///
/// If `Safe` is set to false, the policy may lead to reallocate memory currently in-use.
#[zenoh_macros::unstable_doc]
#[derive(Clone, Copy)]
pub struct GarbageCollect<InnerPolicy = JustAlloc, AltPolicy = JustAlloc, Safe = ConstBool<true>> {
    inner_policy: InnerPolicy,
    alt_policy: AltPolicy,
    safe: Safe,
}
impl<InnerPolicy, AltPolicy, Safe> GarbageCollect<InnerPolicy, AltPolicy, Safe> {
    /// Creates a new `GarbageCollect` policy.
    pub fn new(inner_policy: InnerPolicy, alt_policy: AltPolicy, safe: Safe) -> Self {
        Self {
            inner_policy,
            alt_policy,
            safe,
        }
    }
}
impl<Backend, InnerPolicy, AltPolicy, Safe> AllocPolicy<Backend>
    for GarbageCollect<InnerPolicy, AltPolicy, Safe>
where
    Backend: ShmProviderBackend,
    InnerPolicy: AllocPolicy<Backend>,
    AltPolicy: AllocPolicy<Backend>,
    Safe: PolicyValue<bool>,
{
    fn alloc(&self, layout: &MemoryLayout, provider: &ShmProvider<Backend>) -> ChunkAllocResult {
        let result = self.inner_policy.alloc(layout, provider);
        if result.is_err() {
            // try to alloc again only if GC managed to reclaim big enough chunk
            let collected = if self.safe.get() {
                provider.garbage_collect()
            } else {
                // SAFETY: the policy doesn't implement `SafePolicy` in this case, so the user
                // must enforce the safety himself
                unsafe { provider.garbage_collect_unsafe() }
            };
            if collected >= layout.size().get() {
                return self.alt_policy.alloc(layout, provider);
            }
        }
        result
    }
}
unsafe impl<InnerPolicy: SafePolicy, AltPolicy: SafePolicy> SafePolicy
    for GarbageCollect<InnerPolicy, AltPolicy, ConstBool<true>>
{
}
impl<InnerPolicy: ConstPolicy, AltPolicy: ConstPolicy, Safe: ConstPolicy> ConstPolicy
    for GarbageCollect<InnerPolicy, AltPolicy, Safe>
{
    const NEW: Self = Self {
        inner_policy: InnerPolicy::NEW,
        alt_policy: AltPolicy::NEW,
        safe: Safe::NEW,
    };
}

/// Defragmenting policy.
///
/// Try to defragment if allocation failed and allocate again
/// if the largest defragmented chuk is not smaller than the one required
#[zenoh_macros::unstable_doc]
#[derive(Clone, Copy)]
pub struct Defragment<InnerPolicy = JustAlloc, AltPolicy = JustAlloc> {
    inner_policy: InnerPolicy,
    alt_policy: AltPolicy,
}
impl<InnerPolicy, AltPolicy> Defragment<InnerPolicy, AltPolicy> {
    /// Creates a new `Defragment` policy.
    pub fn new(inner_policy: InnerPolicy, alt_policy: AltPolicy) -> Self {
        Self {
            inner_policy,
            alt_policy,
        }
    }
}
impl<Backend, InnerPolicy, AltPolicy> AllocPolicy<Backend> for Defragment<InnerPolicy, AltPolicy>
where
    Backend: ShmProviderBackend,
    InnerPolicy: AllocPolicy<Backend>,
    AltPolicy: AllocPolicy<Backend>,
{
    fn alloc(&self, layout: &MemoryLayout, provider: &ShmProvider<Backend>) -> ChunkAllocResult {
        let result = self.inner_policy.alloc(layout, provider);
        if let Err(ZAllocError::NeedDefragment) = result {
            // try to alloc again only if big enough chunk was defragmented
            if provider.defragment() >= layout.size().get() {
                return self.alt_policy.alloc(layout, provider);
            }
        }
        result
    }
}
unsafe impl<InnerPolicy: SafePolicy, AltPolicy: SafePolicy> SafePolicy
    for Defragment<InnerPolicy, AltPolicy>
{
}
impl<InnerPolicy: ConstPolicy, AltPolicy: ConstPolicy> ConstPolicy
    for Defragment<InnerPolicy, AltPolicy>
{
    const NEW: Self = Self {
        inner_policy: InnerPolicy::NEW,
        alt_policy: AltPolicy::NEW,
    };
}

/// Deallocating policy.
///
/// Forcibly deallocate up to N buffers until allocation succeeds.
///
/// This policy is unsafe as it may lead to reallocate memory currently in-use.
#[zenoh_macros::unstable_doc]
#[derive(Clone, Copy)]
pub struct Deallocate<Limit, InnerPolicy = JustAlloc, AltPolicy = InnerPolicy> {
    limit: Limit,
    inner_policy: InnerPolicy,
    alt_policy: AltPolicy,
}
impl<Limit, InnerPolicy, AltPolicy> Deallocate<Limit, InnerPolicy, AltPolicy> {
    /// Creates a new `Deallocate` policy.
    pub fn new(limit: Limit, inner_policy: InnerPolicy, alt_policy: AltPolicy) -> Self {
        Self {
            limit,
            inner_policy,
            alt_policy,
        }
    }
}
impl<Backend, Limit, InnerPolicy, AltPolicy> AllocPolicy<Backend>
    for Deallocate<Limit, InnerPolicy, AltPolicy>
where
    Backend: ShmProviderBackend,
    Limit: PolicyValue<usize>,
    InnerPolicy: AllocPolicy<Backend>,
    AltPolicy: AllocPolicy<Backend>,
{
    fn alloc(&self, layout: &MemoryLayout, provider: &ShmProvider<Backend>) -> ChunkAllocResult {
        for _ in 0..self.limit.get() {
            match self.inner_policy.alloc(layout, provider) {
                res @ (Err(ZAllocError::NeedDefragment) | Err(ZAllocError::OutOfMemory)) => {
                    let Some(chunk) = zlock!(provider.busy_list).pop() else {
                        return res;
                    };
                    provider.backend.free(&chunk.descriptor());
                }
                res => return res,
            }
        }
        self.alt_policy.alloc(layout, provider)
    }
}
impl<Limit: ConstPolicy, InnerPolicy: ConstPolicy, AltPolicy: ConstPolicy> ConstPolicy
    for Deallocate<Limit, InnerPolicy, AltPolicy>
{
    const NEW: Self = Self {
        limit: Limit::NEW,
        inner_policy: InnerPolicy::NEW,
        alt_policy: AltPolicy::NEW,
    };
}

/// Blocking allocation policy.
///
/// This policy will block until the allocation succeeds.
/// Both sync and async modes available.
#[zenoh_macros::unstable_doc]
#[derive(Clone, Copy)]
pub struct BlockOn<InnerPolicy = JustAlloc> {
    inner_policy: InnerPolicy,
}
impl<InnerPolicy> BlockOn<InnerPolicy> {
    /// Creates a new `BlockOn` policy.
    pub fn new(inner_policy: InnerPolicy) -> Self {
        Self { inner_policy }
    }
}
#[async_trait]
impl<Backend, InnerPolicy> AsyncAllocPolicy<Backend> for BlockOn<InnerPolicy>
where
    Backend: ShmProviderBackend + Sync,
    InnerPolicy: AllocPolicy<Backend> + Send + Sync,
{
    async fn alloc_async(
        &self,
        layout: &MemoryLayout,
        provider: &ShmProvider<Backend>,
    ) -> ChunkAllocResult {
        loop {
            match self.inner_policy.alloc(layout, provider) {
                Err(ZAllocError::NeedDefragment) | Err(ZAllocError::OutOfMemory) => {
                    // TODO: implement provider's async signalling instead of this!
                    tokio::time::sleep(Duration::from_millis(1)).await;
                }
                other_result => {
                    return other_result;
                }
            }
        }
    }
}
unsafe impl<InnerPolicy: SafePolicy> SafePolicy for BlockOn<InnerPolicy> {}
impl<Backend, InnerPolicy> AllocPolicy<Backend> for BlockOn<InnerPolicy>
where
    Backend: ShmProviderBackend,
    InnerPolicy: AllocPolicy<Backend>,
{
    fn alloc(&self, layout: &MemoryLayout, provider: &ShmProvider<Backend>) -> ChunkAllocResult {
        loop {
            match self.inner_policy.alloc(layout, provider) {
                Err(ZAllocError::NeedDefragment) | Err(ZAllocError::OutOfMemory) => {
                    // TODO: implement provider's async signalling instead of this!
                    std::thread::sleep(Duration::from_millis(1));
                }
                other_result => {
                    return other_result;
                }
            }
        }
    }
}
impl<InnerPolicy: ConstPolicy> ConstPolicy for BlockOn<InnerPolicy> {
    const NEW: Self = Self {
        inner_policy: InnerPolicy::NEW,
    };
}

#[zenoh_macros::unstable_doc]
pub struct AllocBuilder<'a, Backend, Layout, Policy = JustAlloc> {
    provider: &'a ShmProvider<Backend>,
    layout: Layout,
    policy: Policy,
}

// Generic impl
impl<'a, Backend, Layout, Policy> AllocBuilder<'a, Backend, Layout, Policy> {
    /// Set the allocation policy
    #[zenoh_macros::unstable_doc]
    pub fn with_policy<OtherPolicy: SafePolicy + ConstPolicy>(
        self,
    ) -> AllocBuilder<'a, Backend, Layout, OtherPolicy> {
        // SAFETY: the policy is safe
        unsafe { self.with_runtime_policy(OtherPolicy::NEW) }
    }

    /// Set the unsafe allocation policy
    ///
    /// # Safety
    ///
    /// The user must ensure its use of allocated buffers are safe taking into account the policy.
    #[zenoh_macros::unstable_doc]
    pub unsafe fn with_unsafe_policy<OtherPolicy: ConstPolicy>(
        self,
    ) -> AllocBuilder<'a, Backend, Layout, OtherPolicy> {
        // SAFETY: same function contract
        unsafe { self.with_runtime_policy(OtherPolicy::NEW) }
    }

    /// Set the unsafe allocation policy at runtime
    ///
    /// # Safety
    ///
    /// The user must ensure its use of allocated buffers are safe taking into account the policy.
    #[zenoh_macros::unstable_doc]
    pub unsafe fn with_runtime_policy<OtherPolicy>(
        self,
        policy: OtherPolicy,
    ) -> AllocBuilder<'a, Backend, Layout, OtherPolicy> {
        AllocBuilder {
            provider: self.provider,
            layout: self.layout,
            policy,
        }
    }

    fn layout_policy(self) -> Result<(PrecomputedLayout<'a, Backend, Layout>, Policy), ZLayoutError>
    where
        Backend: ShmProviderBackend,
        Layout: AllocLayout,
    {
        let layout = PrecomputedLayout::new(self.provider, self.layout.memory_layout()?)?;
        Ok((layout, self.policy))
    }
}

impl<'a, Backend: ShmProviderBackend, Layout: TryInto<MemoryLayout>>
    AllocBuilder<'a, Backend, Layout>
where
    Layout::Error: Into<ZLayoutError>,
{
    /// Try to build an allocation layout
    #[deprecated(
        since = "1.5.3",
        note = "Please use `ShmProvider::alloc_layout` method instead"
    )]
    #[zenoh_macros::unstable_doc]
    pub fn into_layout(self) -> Result<PrecomputedLayout<'a, Backend, Layout>, ZLayoutError> {
        PrecomputedLayout::new(self.provider, self.layout.try_into().map_err(Into::into)?)
    }
}

impl<Backend, Layout: AllocLayout, Policy> Resolvable
    for AllocBuilder<'_, Backend, Layout, Policy>
{
    type To = Result<Layout::Buffer, ZLayoutAllocError>;
}

impl<Backend: ShmProviderBackend, Layout: AllocLayout, Policy: AllocPolicy<Backend>> Wait
    for AllocBuilder<'_, Backend, Layout, Policy>
{
    fn wait(self) -> <Self as Resolvable>::To {
        let (layout, policy) = self.layout_policy()?;
        // SAFETY: same contract used to set the policy
        Ok(unsafe { layout.alloc().with_runtime_policy(policy) }.wait()?)
    }
}

impl<
        'a,
        Backend: ShmProviderBackend + Sync,
        Layout: AllocLayout + Send + Sync + 'a,
        Policy: AsyncAllocPolicy<Backend> + Sync + 'a,
    > IntoFuture for AllocBuilder<'a, Backend, Layout, Policy>
{
    type Output = <Self as Resolvable>::To;
    type IntoFuture = Pin<Box<dyn Future<Output = <Self as IntoFuture>::Output> + 'a + Send>>;

    fn into_future(self) -> Self::IntoFuture {
        Box::pin(async move {
            let (layout, policy) = self.layout_policy()?;
            // SAFETY: same contract used to set the policy
            Ok(unsafe { layout.alloc().with_runtime_policy(policy) }.await?)
        })
    }
}

/// Builder for making allocations with instant layout calculation
#[zenoh_macros::unstable_doc]
pub struct PrecomputedAllocBuilder<'b, 'a: 'b, Backend, Layout, Policy = JustAlloc> {
    layout: &'b PrecomputedLayout<'a, Backend, Layout>,
    policy: Policy,
}

// Generic impl
impl<'b, 'a: 'b, Backend, Layout, Policy> PrecomputedAllocBuilder<'b, 'a, Backend, Layout, Policy> {
    /// Set the allocation policy
    #[zenoh_macros::unstable_doc]
    pub fn with_policy<OtherPolicy: ConstPolicy>(
        self,
    ) -> PrecomputedAllocBuilder<'b, 'a, Backend, Layout, OtherPolicy> {
        // SAFETY: the policy is safe
        unsafe { self.with_runtime_policy(OtherPolicy::NEW) }
    }

    /// Set the unsafe allocation policy
    ///
    /// # Safety
    ///
    /// The user must ensure its use of allocated buffers are safe taking into account the policy.
    #[zenoh_macros::unstable_doc]
    pub unsafe fn with_unsafe_policy<OtherPolicy: ConstPolicy>(
        self,
    ) -> PrecomputedAllocBuilder<'b, 'a, Backend, Layout, OtherPolicy> {
        // SAFETY: same function contract
        unsafe { self.with_runtime_policy(OtherPolicy::NEW) }
    }

    /// Set the unsafe allocation policy at runtime
    ///
    /// # Safety
    ///
    /// The user must ensure its use of allocated buffers are safe taking into account the policy.
    #[zenoh_macros::unstable_doc]
    pub unsafe fn with_runtime_policy<OtherPolicy>(
        self,
        policy: OtherPolicy,
    ) -> PrecomputedAllocBuilder<'b, 'a, Backend, Layout, OtherPolicy> {
        PrecomputedAllocBuilder {
            layout: self.layout,
            policy,
        }
    }
}

impl<Backend, Layout: AllocLayout, Policy> Resolvable
    for PrecomputedAllocBuilder<'_, '_, Backend, Layout, Policy>
{
    type To = Result<Layout::Buffer, ZAllocError>;
}

impl<Backend: ShmProviderBackend, Layout: AllocLayout, Policy: AllocPolicy<Backend>> Wait
    for PrecomputedAllocBuilder<'_, '_, Backend, Layout, Policy>
{
    fn wait(self) -> <Self as Resolvable>::To {
        let buffer = self.layout.provider.alloc_inner(
            self.layout.size,
            &self.layout.provider_layout,
            &self.policy,
        )?;
        // SAFETY: the buffer has been allocated with the requested layout
        Ok(unsafe { Layout::wrap_buffer(buffer) })
    }
}

impl<
        'b,
        'a: 'b,
        Backend: ShmProviderBackend + Sync,
        Layout: AllocLayout + Sync,
        Policy: AsyncAllocPolicy<Backend> + Sync + 'b,
    > IntoFuture for PrecomputedAllocBuilder<'b, 'a, Backend, Layout, Policy>
{
    type Output = <Self as Resolvable>::To;
    type IntoFuture = Pin<Box<dyn Future<Output = <Self as IntoFuture>::Output> + 'b + Send>>;

    fn into_future(self) -> Self::IntoFuture {
        Box::pin(async move {
            let buffer = self
                .layout
                .provider
                .alloc_inner_async(self.layout.size, &self.layout.provider_layout, &self.policy)
                .await?;
            // SAFETY: the buffer has been allocated with the requested layout
            Ok(unsafe { Layout::wrap_buffer(buffer) })
        })
    }
}

#[zenoh_macros::unstable_doc]
pub struct ShmProviderBuilder;
impl ShmProviderBuilder {
    /// Set the backend
    #[zenoh_macros::unstable_doc]
    pub fn backend<Backend: ShmProviderBackend>(
        backend: Backend,
    ) -> ShmProviderBuilderBackend<Backend> {
        ShmProviderBuilderBackend { backend }
    }

    /// Set the default backend
    #[zenoh_macros::unstable_doc]
    pub fn default_backend<Layout>(layout: Layout) -> ShmProviderBuilderWithDefaultBackend<Layout> {
        ShmProviderBuilderWithDefaultBackend { layout }
    }
}

#[zenoh_macros::unstable_doc]
pub struct ShmProviderBuilderBackend<Backend>
where
    Backend: ShmProviderBackend,
{
    backend: Backend,
}
#[zenoh_macros::unstable_doc]
impl<Backend> Resolvable for ShmProviderBuilderBackend<Backend>
where
    Backend: ShmProviderBackend,
{
    type To = ShmProvider<Backend>;
}

#[zenoh_macros::unstable_doc]
impl<Backend> Wait for ShmProviderBuilderBackend<Backend>
where
    Backend: ShmProviderBackend,
{
    /// build ShmProvider
    fn wait(self) -> <Self as Resolvable>::To {
        ShmProvider::new(self.backend)
    }
}

#[zenoh_macros::unstable_doc]
pub struct ShmProviderBuilderWithDefaultBackend<Layout> {
    layout: Layout,
}

#[zenoh_macros::unstable_doc]
impl<Layout> Resolvable for ShmProviderBuilderWithDefaultBackend<Layout> {
    type To = ZResult<ShmProvider<PosixShmProviderBackend>>;
}

#[zenoh_macros::unstable_doc]
impl<Layout: TryInto<MemoryLayout>> Wait for ShmProviderBuilderWithDefaultBackend<Layout>
where
    Layout::Error: Error,
    PosixShmProviderBackendBuilder<Layout>:
        Resolvable<To = ZResult<PosixShmProviderBackend>> + Wait,
{
    /// build ShmProvider
    fn wait(self) -> <Self as Resolvable>::To {
        let backend = PosixShmProviderBackend::builder(self.layout).wait()?;
        Ok(ShmProvider::new(backend))
    }
}

#[derive(Debug)]
struct BusyChunk {
    metadata: AllocatedMetadataDescriptor,
}

impl BusyChunk {
    fn new(metadata: AllocatedMetadataDescriptor) -> Self {
        Self { metadata }
    }

    fn descriptor(&self) -> ChunkDescriptor {
        self.metadata.header().data_descriptor()
    }
}

/// A generalized interface for shared memory data sources
#[zenoh_macros::unstable_doc]
#[derive(Debug)]
pub struct ShmProvider<Backend> {
    backend: Backend,
    busy_list: Mutex<Vec<BusyChunk>>,
}

impl<Backend: Default> Default for ShmProvider<Backend> {
    fn default() -> Self {
        Self::new(Backend::default())
    }
}

impl<Backend> ShmProvider<Backend>
where
    Backend: ShmProviderBackend,
{
    /// Allocates a buffer with a given memory layout.
    #[zenoh_macros::unstable_doc]
    pub fn alloc<Layout>(&self, layout: Layout) -> AllocBuilder<'_, Backend, Layout> {
        AllocBuilder {
            provider: self,
            layout,
            policy: JustAlloc,
        }
    }

    /// Precompute the actual layout for an allocation.
    #[zenoh_macros::unstable_doc]
    pub fn alloc_layout<Layout: TryInto<MemoryLayout>>(
        &self,
        layout: Layout,
    ) -> Result<PrecomputedLayout<'_, Backend, Layout>, ZLayoutError>
    where
        Layout::Error: Into<ZLayoutError>,
    {
        PrecomputedLayout::new(self, layout.try_into().map_err(Into::into)?)
    }
    /// Defragment memory
    #[zenoh_macros::unstable_doc]
    pub fn defragment(&self) -> usize {
        self.backend.defragment()
    }

    /// Map externally-allocated chunk into ZShmMut.
    ///
    /// This method is designed to be used with push data sources.
    /// Remember that chunk's len may be >= len!
    #[zenoh_macros::unstable_doc]
    pub fn map(&self, chunk: AllocatedChunk, len: usize) -> Result<ZShmMut, ZAllocError> {
        let len = len.try_into().map_err(|_| ZAllocError::Other)?;

        // allocate resources for SHM buffer
        let (allocated_metadata, confirmed_metadata) = Self::alloc_resources()?;

        // wrap everything to ShmBufInner
        let wrapped = self.wrap(chunk, len, allocated_metadata, confirmed_metadata);
        Ok(unsafe { ZShmMut::new_unchecked(wrapped) })
    }

    #[zenoh_macros::unstable_doc]
    fn garbage_collect_impl<const SAFE: bool>(&self) -> usize {
        fn retain_unordered<T>(vec: &mut Vec<T>, mut f: impl FnMut(&T) -> bool) {
            let mut i = 0;
            while i < vec.len() {
                if f(&vec[i]) {
                    i += 1;
                } else {
                    vec.swap_remove(i); // move last into place of vec[i]
                                        // don't increment i: need to test the swapped-in element
                }
            }
        }

        tracing::trace!("Running Garbage Collector");
        let mut largest = 0usize;
        retain_unordered(&mut zlock!(self.busy_list), |chunk| {
            let header = chunk.metadata.header();
            if header.refcount.load(Ordering::SeqCst) == 0
                || (!SAFE && header.watchdog_invalidated.load(Ordering::SeqCst))
            {
                tracing::trace!("Garbage Collecting Chunk: {:?}", chunk);
                let descriptor_to_free = chunk.descriptor();
                self.backend.free(&descriptor_to_free);
                largest = largest.max(descriptor_to_free.len.get());
                return false;
            }
            true
        });
        largest
    }

    /// Try to collect free chunks.
    ///
    /// Returns the size of largest collected chunk
    #[zenoh_macros::unstable_doc]
    pub fn garbage_collect(&self) -> usize {
        self.garbage_collect_impl::<true>()
    }

    /// Try to collect free chunks.
    ///
    /// Returns the size of largest collected chunk
    ///
    /// # Safety
    ///
    /// User must ensure there is no data races with collected chunks, as some of them may still be in-use.
    #[zenoh_macros::unstable_doc]
    pub unsafe fn garbage_collect_unsafe(&self) -> usize {
        self.garbage_collect_impl::<false>()
    }

    /// Bytes available for use
    #[zenoh_macros::unstable_doc]
    pub fn available(&self) -> usize {
        self.backend.available()
    }
}

// PRIVATE impls
impl<Backend> ShmProvider<Backend> {
    fn new(backend: Backend) -> Self {
        Self {
            backend,
            busy_list: Default::default(),
        }
    }
}

impl<Backend> ShmProvider<Backend>
where
    Backend: ShmProviderBackend,
{
    fn alloc_inner<Policy>(
        &self,
        size: NonZeroUsize,
        layout: &MemoryLayout,
        policy: &Policy,
    ) -> Result<ZShmMut, ZAllocError>
    where
        Policy: AllocPolicy<Backend>,
    {
        // allocate resources for SHM buffer
        let (allocated_metadata, confirmed_metadata) = Self::alloc_resources()?;

        // allocate data chunk
        // Perform actions depending on the Policy
        // NOTE: it is necessary to properly map this chunk OR free it if mapping fails!
        // Don't loose this chunk as it leads to memory leak at the backend side!
        // NOTE: self.backend.alloc(len) returns chunk with len >= required len,
        // and it is necessary to handle that properly and pass this len to corresponding free(...)
        let chunk = policy.alloc(layout, self)?;

        // wrap allocated chunk to ShmBufInner
        let wrapped = self.wrap(chunk, size, allocated_metadata, confirmed_metadata);
        Ok(unsafe { ZShmMut::new_unchecked(wrapped) })
    }

    fn alloc_resources() -> Result<(AllocatedMetadataDescriptor, ConfirmedDescriptor), ZAllocError>
    {
        // allocate metadata
        let allocated_metadata = GLOBAL_METADATA_STORAGE.read().allocate()?;

        // add watchdog to confirmator
        let confirmed_metadata = GLOBAL_CONFIRMATOR.read().add(allocated_metadata.clone());

        Ok((allocated_metadata, confirmed_metadata))
    }

    fn wrap(
        &self,
        chunk: AllocatedChunk,
        len: NonZeroUsize,
        allocated_metadata: AllocatedMetadataDescriptor,
        confirmed_metadata: ConfirmedDescriptor,
    ) -> ShmBufInner {
        // write additional metadata
        // chunk descriptor
        allocated_metadata
            .header()
            .set_data_descriptor(&chunk.descriptor);
        // protocol
        allocated_metadata
            .header()
            .protocol
            .store(self.backend.id(), Ordering::Relaxed);

        // add watchdog to validator
        GLOBAL_VALIDATOR
            .read()
            .add(confirmed_metadata.owned.clone());

        // Create buffer's info
        let info = ShmBufInfo::new(
            len,
            MetadataDescriptor::from(&confirmed_metadata.owned),
            allocated_metadata
                .header()
                .generation
                .load(Ordering::SeqCst),
        );

        // Create buffer
        let shmb = ShmBufInner {
            metadata: confirmed_metadata,
            buf: chunk.data,
            info,
        };

        // Create and store busy chunk
        zlock!(self.busy_list).push(BusyChunk::new(allocated_metadata));

        shmb
    }
}

// PRIVATE impls for Sync backend
impl<Backend> ShmProvider<Backend>
where
    Backend: ShmProviderBackend + Sync,
{
    async fn alloc_inner_async<Policy>(
        &self,
        size: NonZeroUsize,
        backend_layout: &MemoryLayout,
        policy: &Policy,
    ) -> Result<ZShmMut, ZAllocError>
    where
        Policy: AsyncAllocPolicy<Backend>,
    {
        // allocate resources for SHM buffer
        let (allocated_metadata, confirmed_metadata) = Self::alloc_resources()?;

        // allocate data chunk
        // Perform actions depending on the Policy
        // NOTE: it is necessary to properly map this chunk OR free it if mapping fails!
        // Don't loose this chunk as it leads to memory leak at the backend side!
        // NOTE: self.backend.alloc(len) returns chunk with len >= required len,
        // and it is necessary to handle that properly and pass this len to corresponding free(...)
        let chunk = policy.alloc_async(backend_layout, self).await?;

        // wrap allocated chunk to ShmBufInner
        let wrapped = self.wrap(chunk, size, allocated_metadata, confirmed_metadata);
        Ok(unsafe { ZShmMut::new_unchecked(wrapped) })
    }
}
