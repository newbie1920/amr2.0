/**
 * Store Registry — Breaks circular dependencies between stores.
 * 
 * Pattern: Each store registers itself after creation, and other stores
 * access it via this registry. This avoids ESM circular import issues.
 */

const registry = {};

export function registerStore(name, store) {
  registry[name] = store;
}

export function getStore(name) {
  return registry[name];
}

export function getRobotStoreState() {
  const store = registry.robotStore;
  if (!store) {
    console.warn('[StoreRegistry] robotStore not yet registered');
    return { robots: {}, velocityMuxes: {} };
  }
  return store.getState();
}

export function getNavStoreState() {
  const store = registry.navStore;
  if (!store) {
    console.warn('[StoreRegistry] navStore not yet registered');
    return { appNavigationSessions: {}, navComputationBusy: {} };
  }
  return store.getState();
}

export function getMapStoreState() {
  const store = registry.mapStore;
  if (!store) return { mapperInstances: {}, occupancyGrid: {} };
  return store.getState();
}

export function getDWAStoreState() {
  const store = registry.dwaStore;
  if (!store) return { dwaConfig: {} };
  return store.getState();
}
