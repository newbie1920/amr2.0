import * as Comlink from 'comlink';
import NavWorker from './navWorker.js?worker';

let navWorkerApi = null;
if (typeof Worker !== 'undefined') {
  navWorkerApi = Comlink.wrap(new NavWorker());
}

export { navWorkerApi };
