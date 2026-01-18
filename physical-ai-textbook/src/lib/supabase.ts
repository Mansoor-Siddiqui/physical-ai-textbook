import { createClient } from '@supabase/supabase-js';

const supabaseUrl = 'https://kwfbazivsqdfbtjgehjr.supabase.co';
const supabaseAnonKey = 'sb_publishable_w4O3Qs6xahFhly0_n7Qiwg_wL3uEnCM';

export const supabase = createClient(supabaseUrl, supabaseAnonKey);
