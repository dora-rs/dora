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
    collections::BTreeMap,
    sync::{Arc, RwLock},
};

use static_init::dynamic;
use zenoh_result::{bail, ZResult};

use crate::{
    api::{
        client::{shm_client::ShmClient, shm_segment::ShmSegment},
        common::{types::ProtocolID, with_id::WithProtocolID},
        protocol_implementations::posix::posix_shm_client::PosixShmClient,
    },
    reader::{ClientStorage, GlobalDataSegmentId},
};

#[dynamic(lazy, drop)]
/// A global lazily-initialized SHM client storage.
///
/// When initialized, contains default client set, see [with_default_client_set](ShmClientSetBuilder::with_default_client_set)
#[zenoh_macros::unstable_doc]
pub static mut GLOBAL_CLIENT_STORAGE: Arc<ShmClientStorage> = Arc::new(
    ShmClientStorage::builder()
        .with_default_client_set()
        .build(),
);

/// Builder to create new client storages
#[zenoh_macros::unstable_doc]
pub struct ShmClientSetBuilder;

impl ShmClientSetBuilder {
    /// Add client to the storage (without including the default client set)
    #[zenoh_macros::unstable_doc]
    pub fn with_client(self, client: Arc<dyn ShmClient>) -> ShmClientStorageBuilder {
        let clients = BTreeMap::from([(client.id(), client)]);
        ShmClientStorageBuilder::new(clients)
    }

    /// Add list of clients to the storage (without including the default client set)
    #[zenoh_macros::unstable_doc]
    pub fn with_clients(self, clients: &[Arc<dyn ShmClient>]) -> ShmClientStorageBuilder {
        let clients = clients
            .iter()
            .cloned()
            .map(|client| (client.id(), client))
            .collect();
        ShmClientStorageBuilder::new(clients)
    }

    /// Include default clients
    #[zenoh_macros::unstable_doc]
    pub fn with_default_client_set(self) -> ShmClientStorageBuilder {
        let client = PosixShmClient {};
        let clients = BTreeMap::from([(client.id(), Arc::new(client) as Arc<dyn ShmClient>)]);
        ShmClientStorageBuilder::new(clients)
    }
}

#[zenoh_macros::unstable_doc]
pub struct ShmClientStorageBuilder {
    clients: BTreeMap<ProtocolID, Arc<dyn ShmClient>>,
}

impl ShmClientStorageBuilder {
    fn new(clients: BTreeMap<ProtocolID, Arc<dyn ShmClient>>) -> Self {
        Self { clients }
    }

    /// Add client to the storage
    #[zenoh_macros::unstable_doc]
    pub fn with_client(mut self, client: Arc<dyn ShmClient>) -> ZResult<Self> {
        let id = client.id();
        match self.clients.entry(id) {
            std::collections::btree_map::Entry::Occupied(occupied) => {
                bail!("Client already exists for id {id}: {:?}!", occupied)
            }
            std::collections::btree_map::Entry::Vacant(vacant) => {
                vacant.insert(client as Arc<dyn ShmClient>);
                Ok(self)
            }
        }
    }

    /// Add list of clients to the storage
    #[zenoh_macros::unstable_doc]
    pub fn with_clients(mut self, clients: &[Arc<dyn ShmClient>]) -> Self {
        self.clients
            .extend(clients.iter().cloned().map(|client| (client.id(), client)));
        self
    }

    /// Build the storage with parameters specified on previous step
    #[zenoh_macros::unstable_doc]
    pub fn build(self) -> ShmClientStorage {
        ShmClientStorage::new(self.clients)
    }
}

/// A storage for SHM clients.
///
/// Runtime or Session constructed with instance of this type gets capabilities to read
/// SHM buffers for Protocols added to this instance.
#[zenoh_macros::unstable_doc]
#[derive(Debug)]
pub struct ShmClientStorage {
    pub(crate) clients: ClientStorage<Arc<dyn ShmClient>>,
    pub(crate) segments: RwLock<BTreeMap<GlobalDataSegmentId, Arc<dyn ShmSegment>>>,
}

impl Eq for ShmClientStorage {}

impl PartialEq for ShmClientStorage {
    fn eq(&self, other: &Self) -> bool {
        std::ptr::eq(self, other)
    }
}

impl ShmClientStorage {
    /// Get the builder to construct a new storage
    #[zenoh_macros::unstable_doc]
    pub fn builder() -> ShmClientSetBuilder {
        ShmClientSetBuilder
    }

    /// Get the list of supported SHM protocols.
    #[zenoh_macros::unstable_doc]
    pub fn supported_protocols(&self) -> Vec<ProtocolID> {
        self.clients.get_clients().keys().copied().collect()
    }

    fn new(clients: BTreeMap<ProtocolID, Arc<dyn ShmClient>>) -> Self {
        Self {
            clients: ClientStorage::new(clients),
            segments: RwLock::default(),
        }
    }
}
